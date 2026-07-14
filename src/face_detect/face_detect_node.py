#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸检测节点（集成 D415 相机）
- pyrealsense2 抓帧 → RKNN YOLOv8-face 推理
- 发布:
    /face/detections        (face_msgs/FaceDetectionArray)  ★主输出★
    /face/image_annotated   (sensor_msgs/Image)             可视化（可关闭）
"""

import os
import time
import threading
import queue

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

from face_msgs.msg import FaceDetection, FaceDetectionArray
from face_detect.rknn_detector import RKNNFaceDetector


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')

        # ---- 参数 ----
        self.declare_parameter('model_path',
            '/home/cat/ros2_ws/models/yolov8n-face.rknn')
        self.declare_parameter('detections_topic', '/face/detections')
        self.declare_parameter('annotated_topic',  '/face/image_annotated')
        self.declare_parameter('frame_id',         'camera_color_optical_frame')

        self.declare_parameter('camera_width',  640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps',    30)
        self.declare_parameter('camera_serial', '')

        self.declare_parameter('conf_threshold',    0.5)
        self.declare_parameter('iou_threshold',     0.45)
        self.declare_parameter('crop_padding',      0.15)
        self.declare_parameter('min_face_size',     40)
        self.declare_parameter('publish_annotated', True)

        self.model_path        = self.get_parameter('model_path').value
        self.det_topic         = self.get_parameter('detections_topic').value
        self.annotated_topic   = self.get_parameter('annotated_topic').value
        self.frame_id          = self.get_parameter('frame_id').value

        self.cam_w             = int(self.get_parameter('camera_width').value)
        self.cam_h             = int(self.get_parameter('camera_height').value)
        self.cam_fps           = int(self.get_parameter('camera_fps').value)
        self.cam_serial        = self.get_parameter('camera_serial').value

        conf_thr               = float(self.get_parameter('conf_threshold').value)
        iou_thr                = float(self.get_parameter('iou_threshold').value)
        self.crop_padding      = float(self.get_parameter('crop_padding').value)
        self.min_face_size     = int(self.get_parameter('min_face_size').value)
        self.publish_annotated = bool(self.get_parameter('publish_annotated').value)

        # ---- 加载 RKNN ----
        if not os.path.exists(self.model_path):
            self.get_logger().fatal(f'模型文件不存在: {self.model_path}')
            raise FileNotFoundError(self.model_path)
        self.get_logger().info(f'加载 RKNN 模型: {self.model_path}')
        self.detector = RKNNFaceDetector(
            model_path=self.model_path,
            conf_thr=conf_thr,
            iou_thr=iou_thr,
        )
        self.get_logger().info('✅ RKNN 模型加载成功')

        # ---- 启动 D415 ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if self.cam_serial:
            cfg.enable_device(self.cam_serial)
            self.get_logger().info(f'指定相机序列号: {self.cam_serial}')
        cfg.enable_stream(rs.stream.color, self.cam_w, self.cam_h,
                          rs.format.bgr8, self.cam_fps)
        try:
            profile = self.pipeline.start(cfg)
            dev = profile.get_device()
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            self.get_logger().info(
                f'✅ 相机已启动: {name} (SN={serial}) '
                f'{self.cam_w}x{self.cam_h}@{self.cam_fps}'
            )
        except Exception as e:
            self.get_logger().fatal(f'启动 D415 失败: {e}')
            raise

        # ---- ROS 发布器（全部 BEST_EFFORT 以适应实时图像流） ----
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.pub_det = self.create_publisher(FaceDetectionArray, self.det_topic, qos)
        self.pub_anno = None
        if self.publish_annotated:
            self.pub_anno = self.create_publisher(Image, self.annotated_topic, qos)

        # ---- 统计 ----
        self._stat_lock = threading.Lock()
        self._n_frames = 0
        self._n_faces  = 0
        self._infer_ms_sum = 0.0
        self._t_last_stat  = time.time()
        self.create_timer(2.0, self._print_stat)

        # ---- 抓帧线程 → 队列 → 推理/发布线程 ----
        # 队列只保留最新一帧，丢旧帧避免堆积
        self._frame_q = queue.Queue(maxsize=1)
        self._running = True

        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._inf_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._cap_thread.start()
        self._inf_thread.start()

        self.get_logger().info(f'发布检测结果: {self.det_topic}')
        if self.publish_annotated:
            self.get_logger().info(f'发布标注图像: {self.annotated_topic}')

    # ---------------- 抓帧线程 ----------------
    def _capture_loop(self):
        while self._running and rclpy.ok():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            except Exception as e:
                self.get_logger().warn(f'wait_for_frames 异常: {e}')
                continue
            color = frames.get_color_frame()
            if not color:
                continue
            frame = np.asanyarray(color.get_data())  # BGR

            # 丢旧帧，保证推理永远跑在最新一帧上
            if self._frame_q.full():
                try:
                    self._frame_q.get_nowait()
                except queue.Empty:
                    pass
            try:
                self._frame_q.put_nowait(frame)
            except queue.Full:
                pass

    # ---------------- 推理 / 发布线程 ----------------
    def _inference_loop(self):
        while self._running and rclpy.ok():
            try:
                frame = self._frame_q.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self._process(frame)
            except Exception as e:
                self.get_logger().error(f'推理处理异常: {e}')

    def _process(self, frame):
        H, W = frame.shape[:2]

        # ---- 推理 ----
        t0 = time.time()
        boxes, scores = self.detector.detect(frame)
        infer_ms = (time.time() - t0) * 1000.0

        stamp = self.get_clock().now().to_msg()

        # ---- 构建 FaceDetectionArray ----
        arr = FaceDetectionArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.frame_id
        arr.image_width = int(W)
        arr.image_height = int(H)

        annotated = frame.copy() if self.pub_anno is not None else None
        n_valid = 0

        for i in range(len(boxes)):
            x1, y1, x2, y2 = [int(v) for v in boxes[i].tolist()]
            # 边界裁剪
            x1 = max(0, min(W - 1, x1))
            y1 = max(0, min(H - 1, y1))
            x2 = max(0, min(W,     x2))
            y2 = max(0, min(H,     y2))
            conf = float(scores[i])

            fw, fh = x2 - x1, y2 - y1
            if fw < self.min_face_size or fh < self.min_face_size:
                continue

            # 外扩 padding 裁剪
            pad_x = int(fw * self.crop_padding)
            pad_y = int(fh * self.crop_padding)
            cx1 = max(0, x1 - pad_x)
            cy1 = max(0, y1 - pad_y)
            cx2 = min(W, x2 + pad_x)
            cy2 = min(H, y2 + pad_y)

            face_img = frame[cy1:cy2, cx1:cx2]
            if face_img.size == 0:
                continue

            try:
                crop_msg = self.bridge.cv2_to_imgmsg(face_img, encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f'cv_bridge 失败: {e}')
                continue
            crop_msg.header.stamp = stamp
            crop_msg.header.frame_id = self.frame_id

            face = FaceDetection()
            face.track_id   = -1
            face.confidence = conf
            face.x1, face.y1, face.x2, face.y2 = x1, y1, x2, y2
            face.center     = Point(x=float((x1 + x2) / 2.0),
                                    y=float((y1 + y2) / 2.0),
                                    z=0.0)
            face.face_crop  = crop_msg
            face.status     = FaceDetection.STATUS_NEW
            face.is_primary = False
            arr.faces.append(face)
            n_valid += 1

            if annotated is not None:
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f'{conf:.2f}', (x1, max(0, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                            cv2.LINE_AA)

        # 发布检测结果（即使空也发，下游可知道"这一帧没人"）
        try:
            self.pub_det.publish(arr)
        except Exception as e:
            self.get_logger().warn(f'发布 detections 失败: {e}')

        if annotated is not None:
            try:
                anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                anno_msg.header.stamp = stamp
                anno_msg.header.frame_id = self.frame_id
                self.pub_anno.publish(anno_msg)
            except Exception as e:
                self.get_logger().warn(f'发布标注图失败: {e}')

        with self._stat_lock:
            self._n_frames     += 1
            self._n_faces      += n_valid
            self._infer_ms_sum += infer_ms

    def _print_stat(self):
        with self._stat_lock:
            n = self._n_frames
            if n == 0:
                return
            dt = time.time() - self._t_last_stat
            fps = n / dt if dt > 0 else 0
            avg_infer = self._infer_ms_sum / n
            avg_faces = self._n_faces / n
            self.get_logger().info(
                f'[STAT] {fps:.1f} FPS | infer {avg_infer:.1f} ms | '
                f'faces/frame {avg_faces:.2f}'
            )
            self._n_frames     = 0
            self._n_faces      = 0
            self._infer_ms_sum = 0.0
            self._t_last_stat  = time.time()
    def destroy_node(self):
        self.get_logger().info('正在关闭...')
        self._running = False
        for t in (getattr(self, '_cap_thread', None),
                  getattr(self, '_inf_thread', None)):
            try:
                if t is not None and t.is_alive():
                    t.join(timeout=2.0)
            except Exception:
                pass
        try:
            self.pipeline.stop()
        except Exception:
            pass
        try:
            self.detector.release()
        except Exception:
            pass
        super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()