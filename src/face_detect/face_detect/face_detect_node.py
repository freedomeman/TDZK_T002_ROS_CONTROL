#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸检测节点（集成 D415 相机 + 深度对齐 + 3D 坐标系 + 主目标稳定选择）
- pyrealsense2 抓帧 (color + aligned depth) → OpenCV Haar Cascade 人脸检测
- 在相机光心建立坐标系:
    X 轴: 深度方向 (前为正)
    Y 轴: 屏幕水平 (右为正)
    Z 轴: 屏幕垂直 (上为正)
- 主目标策略: 画面占比最大的人脸 + 滞回 + 最短保持 + 位置粘性
- 发布:
    /face/target   (face_msgs/FaceTarget)  实时(~30Hz) 主目标 3D 坐标, 给电机
    /face/crop     (face_msgs/FaceCrop)    低频(~8Hz)  主目标裁剪人脸图, 给表情识别
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

from face_msgs.msg import FaceTarget, FaceCrop
from face_detect.face_detector import FaceDetector


class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')

        # ---- 参数 ----
        self.declare_parameter('model_path',
            '/home/cat/ros2_ws/models/yolov8n-face.rknn')
        self.declare_parameter('target_topic', '/face/target')
        self.declare_parameter('crop_topic',   '/face/crop')
        self.declare_parameter('frame_id',     'camera_link')

        self.declare_parameter('camera_width',  640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps',    30)
        self.declare_parameter('camera_serial', '')

        # 深度相关
        self.declare_parameter('enable_depth',     True)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('depth_min',        0.2)
        self.declare_parameter('depth_max',        6.0)

        # 检测相关
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('iou_threshold',  0.45)
        self.declare_parameter('crop_padding',   0.15)
        self.declare_parameter('min_face_size',  40)

        # 人脸图发布频率(Hz)
        self.declare_parameter('face_crop_publish_hz', 8.0)

        # ⭐ 主目标稳定选择相关
        self.declare_parameter('primary_switch_ratio',     1.25)
        self.declare_parameter('primary_min_hold_frames',  5)
        self.declare_parameter('primary_sticky_radius_px', 80)
        self.declare_parameter('primary_lost_tolerance',   3)

        # ---- 读取参数 ----
        self.model_path        = self.get_parameter('model_path').value
        self.target_topic      = self.get_parameter('target_topic').value
        self.crop_topic        = self.get_parameter('crop_topic').value
        self.frame_id          = self.get_parameter('frame_id').value

        self.cam_w             = int(self.get_parameter('camera_width').value)
        self.cam_h             = int(self.get_parameter('camera_height').value)
        self.cam_fps           = int(self.get_parameter('camera_fps').value)
        self.cam_serial        = self.get_parameter('camera_serial').value

        self.enable_depth      = bool(self.get_parameter('enable_depth').value)
        self.depth_patch       = int(self.get_parameter('depth_patch_size').value)
        self.depth_min         = float(self.get_parameter('depth_min').value)
        self.depth_max         = float(self.get_parameter('depth_max').value)

        conf_thr               = float(self.get_parameter('conf_threshold').value)
        iou_thr                = float(self.get_parameter('iou_threshold').value)
        self.crop_padding      = float(self.get_parameter('crop_padding').value)
        self.min_face_size     = int(self.get_parameter('min_face_size').value)

        crop_hz = float(self.get_parameter('face_crop_publish_hz').value)
        if crop_hz <= 0.0:
            crop_hz = 8.0
        self.crop_publish_interval = 1.0 / crop_hz

        self.primary_switch_ratio     = float(self.get_parameter('primary_switch_ratio').value)
        self.primary_min_hold_frames  = int(self.get_parameter('primary_min_hold_frames').value)
        self.primary_sticky_radius_px = float(self.get_parameter('primary_sticky_radius_px').value)
        self.primary_lost_tolerance   = int(self.get_parameter('primary_lost_tolerance').value)

        # ---- 加载 OpenCV Haar Cascade 人脸检测 ----
        self.get_logger().info('加载 OpenCV Haar Cascade 人脸检测...')
        self.detector = FaceDetector(
            conf_thr=conf_thr,
        )
        self.get_logger().info('✅ 人脸检测模型加载成功')

        # ---- 启动 D415 ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if self.cam_serial:
            cfg.enable_device(self.cam_serial)
            self.get_logger().info(f'指定相机序列号: {self.cam_serial}')

        cfg.enable_stream(rs.stream.color, self.cam_w, self.cam_h,
                          rs.format.bgr8, self.cam_fps)
        if self.enable_depth:
            cfg.enable_stream(rs.stream.depth, self.cam_w, self.cam_h,
                              rs.format.z16, self.cam_fps)

        try:
            profile = self.pipeline.start(cfg)
            dev = profile.get_device()
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            self.get_logger().info(
                f'✅ 相机已启动: {name} (SN={serial}) '
                f'{self.cam_w}x{self.cam_h}@{self.cam_fps} '
                f'depth={"ON" if self.enable_depth else "OFF"}'
            )
        except Exception as e:
            self.get_logger().fatal(f'启动 D415 失败: {e}')
            raise

        # ---- 深度对齐 + 内参 + 深度尺度 ----
        self.align = None
        self.depth_intrin = None
        self.depth_scale = 1.0
        if self.enable_depth:
            self.align = rs.align(rs.stream.color)

            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.get_logger().info(f'📏 深度尺度: {self.depth_scale:.6f} m/unit')

            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.depth_intrin = color_profile.get_intrinsics()
            self.get_logger().info(
                f'📷 内参: fx={self.depth_intrin.fx:.2f}, fy={self.depth_intrin.fy:.2f}, '
                f'ppx={self.depth_intrin.ppx:.2f}, ppy={self.depth_intrin.ppy:.2f}'
            )

        # ---- ROS 发布器 ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_target = self.create_publisher(FaceTarget, self.target_topic, qos)
        self.pub_crop   = self.create_publisher(FaceCrop,   self.crop_topic,   qos)
        self.bridge = CvBridge()

        # ---- 主目标跟踪状态 ----
        self.primary_center_px = None       # (cx, cy) 上一帧主目标像素中心
        self.primary_area      = 0.0        # 上一帧主目标 bbox 面积
        self.primary_hold_cnt  = 0          # 已保持帧数
        self.primary_lost_cnt  = 0          # 连续丢失帧数

        # ---- 人脸图节流时间戳 ----
        self.last_crop_publish_time = 0.0

        # ---- 抓帧线程 ----
        self._frame_q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # ---- 推理定时器（尽量快，靠相机 fps 限速）----
        self.timer = self.create_timer(0.001, self._infer_loop)

        # ---- FPS 统计 ----
        self._fps_t0 = time.time()
        self._fps_cnt = 0

        self.get_logger().info(
            f'🚀 face_detect_node 启动完成\n'
            f'   target: {self.target_topic} (~{self.cam_fps}Hz)\n'
            f'   crop:   {self.crop_topic} (~{crop_hz:.1f}Hz)'
        )

    # =========================================================
    # 抓帧线程
    # =========================================================
    def _grab_loop(self):
        while not self._stop.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except Exception as e:
                self.get_logger().warn(f'相机抓帧超时: {e}')
                continue

            if self.align is not None:
                frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color_img = np.asanyarray(color_frame.get_data())

            depth_frame = None
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()

            # 只保留最新一帧
            if self._frame_q.full():
                try:
                    self._frame_q.get_nowait()
                except queue.Empty:
                    pass
            self._frame_q.put((color_img, depth_frame))

    # =========================================================
    # 推理主循环
    # =========================================================
    def _infer_loop(self):
        try:
            color_img, depth_frame = self._frame_q.get_nowait()
        except queue.Empty:
            return

        h, w = color_img.shape[:2]
        stamp = self.get_clock().now().to_msg()

        # ---- 推理 ----
        try:
            boxes, scores = self.detector.detect(color_img)
        except Exception as e:
            self.get_logger().error(f'推理失败: {e}')
            return

        # ---- 过滤过小人脸 ----
        valid = []
        for (x1, y1, x2, y2), sc in zip(boxes, scores):
            bw = x2 - x1
            bh = y2 - y1
            if bw < self.min_face_size or bh < self.min_face_size:
                continue
            valid.append(((x1, y1, x2, y2), sc, bw * bh))

        # ---- 选主目标 ----
        primary = self._select_primary(valid)

        # ---- 发布 FaceTarget（每帧都发）----
        self._publish_target(primary, stamp, depth_frame)

        # ---- 发布 FaceCrop（节流到 ~8Hz，含 has_face=false 心跳）----
        now = time.time()
        if now - self.last_crop_publish_time >= self.crop_publish_interval:
            self._publish_crop(primary, stamp, color_img)
            self.last_crop_publish_time = now

        # ---- FPS 日志 ----
        self._fps_cnt += 1
        dt = time.time() - self._fps_t0
        if dt >= 5.0:
            fps = self._fps_cnt / dt
            self.get_logger().info(
                f'📊 推理 FPS={fps:.1f} | 有效人脸={len(valid)} | '
                f'主目标={"有" if primary else "无"}'
            )
            self._fps_t0 = time.time()
            self._fps_cnt = 0

    # =========================================================
    # 主目标选择（面积最大 + 滞回 + 最短保持 + 位置粘性）
    # =========================================================
    def _select_primary(self, valid):
        """
        valid: [((x1,y1,x2,y2), score, area), ...]
        return: 选中的项 或 None
        """
        if not valid:
            # 允许短暂丢失
            self.primary_lost_cnt += 1
            if self.primary_lost_cnt > self.primary_lost_tolerance:
                self.primary_center_px = None
                self.primary_area = 0.0
                self.primary_hold_cnt = 0
            return None

        self.primary_lost_cnt = 0

        # 按面积排序
        valid_sorted = sorted(valid, key=lambda x: x[2], reverse=True)
        biggest = valid_sorted[0]

        # 若无历史主目标 → 直接选面积最大
        if self.primary_center_px is None:
            self._update_primary_state(biggest)
            return biggest

        # 有历史主目标 → 找最接近历史位置的候选
        px, py = self.primary_center_px
        nearest = None
        nearest_dist = float('inf')
        for item in valid:
            (x1, y1, x2, y2), _, _ = item
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            d = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
            if d < nearest_dist:
                nearest_dist = d
                nearest = item

        # 若最近候选在粘性半径内 → 优先保持
        if nearest is not None and nearest_dist <= self.primary_sticky_radius_px:
            # 但如果 biggest 面积远大于 nearest（超过 switch_ratio），且已保持够久 → 切换
            if (biggest is not nearest
                and self.primary_hold_cnt >= self.primary_min_hold_frames
                and biggest[2] >= nearest[2] * self.primary_switch_ratio):
                self._update_primary_state(biggest)
                return biggest
            # 否则保持 nearest
            self._update_primary_state(nearest)
            return nearest

        # 粘性半径外 → 只有满足滞回条件才切换
        if self.primary_hold_cnt >= self.primary_min_hold_frames:
            self._update_primary_state(biggest)
            return biggest

        # 尚未保持够久 → 继续保持历史（但历史找不到了，只能选 biggest）
        self._update_primary_state(biggest)
        return biggest

    def _update_primary_state(self, item):
        (x1, y1, x2, y2), _, area = item
        new_cx = (x1 + x2) / 2.0
        new_cy = (y1 + y2) / 2.0

        # 判断是否为“同一个”主目标（位置接近）
        if self.primary_center_px is not None:
            px, py = self.primary_center_px
            d = ((new_cx - px) ** 2 + (new_cy - py) ** 2) ** 0.5
            if d <= self.primary_sticky_radius_px:
                self.primary_hold_cnt += 1
            else:
                self.primary_hold_cnt = 1  # 切换到新目标
        else:
            self.primary_hold_cnt = 1

        self.primary_center_px = (new_cx, new_cy)
        self.primary_area = area

    # =========================================================
    # 发布 FaceTarget（3D 坐标, 高频）
    # =========================================================
    def _publish_target(self, primary, stamp, depth_frame):
        msg = FaceTarget()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        if primary is None:
            msg.has_target = False
            msg.center = Point(x=0.0, y=0.0, z=0.0)
            self.pub_target.publish(msg)
            return

        (x1, y1, x2, y2), _, _ = primary
        cx_px = (x1 + x2) / 2.0
        cy_px = (y1 + y2) / 2.0

        # 计算 3D 坐标
        X, Y, Z = 0.0, 0.0, 0.0
        got_3d = False
        if self.enable_depth and depth_frame is not None and self.depth_intrin is not None:
            depth_m = self._sample_depth(depth_frame, int(cx_px), int(cy_px))
            if depth_m > 0.0:
                # deproject: (u,v,depth) → 相机坐标系 (Xc,Yc,Zc)
                # RealSense 相机坐标系: Xc右, Yc下, Zc前
                pt = rs.rs2_deproject_pixel_to_point(
                    self.depth_intrin, [float(cx_px), float(cy_px)], float(depth_m)
                )
                Xc, Yc, Zc = pt[0], pt[1], pt[2]
                # 映射到目标坐标系:
                #   X(深度前) = Zc
                #   Y(水平右) = Xc
                #   Z(垂直上) = -Yc
                X = float(Zc)
                Y = float(-Xc)
                Z = float(-Yc)
                got_3d = True

        if not got_3d:
            # 无深度 → 坐标全 0，但仍标记 has_target=true
            msg.has_target = True
            msg.center = Point(x=0.0, y=0.0, z=0.0)
        else:
            msg.has_target = True
            msg.center = Point(x=X, y=Y, z=Z)

        self.pub_target.publish(msg)

    def _sample_depth(self, depth_frame, cx, cy):
        """在 (cx,cy) 周围 patch 内取有效深度中值，返回米。无效返回 0.0。"""
        try:
            depth_img = np.asanyarray(depth_frame.get_data())
        except Exception:
            return 0.0

        h, w = depth_img.shape[:2]
        r = max(1, self.depth_patch // 2)
        x0 = max(0, cx - r); x1 = min(w, cx + r + 1)
        y0 = max(0, cy - r); y1 = min(h, cy + r + 1)
        patch = depth_img[y0:y1, x0:x1].astype(np.float32)
        patch = patch * self.depth_scale  # 转米
        # 过滤无效值
        valid = patch[(patch >= self.depth_min) & (patch <= self.depth_max)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    # =========================================================
    # 发布 FaceCrop（低频, 给表情识别）
    # =========================================================
    def _publish_crop(self, primary, stamp, color_img):
        msg = FaceCrop()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        if primary is None:
            msg.has_face = False
            # face_crop 保持为默认空 Image
            self.pub_crop.publish(msg)
            return

        (x1, y1, x2, y2), _, _ = primary
        h, w = color_img.shape[:2]

        # 加 padding
        bw = x2 - x1
        bh = y2 - y1
        pad_x = int(bw * self.crop_padding)
        pad_y = int(bh * self.crop_padding)
        cx1 = max(0, int(x1) - pad_x)
        cy1 = max(0, int(y1) - pad_y)
        cx2 = min(w, int(x2) + pad_x)
        cy2 = min(h, int(y2) + pad_y)

        if cx2 <= cx1 or cy2 <= cy1:
            msg.has_face = False
            self.pub_crop.publish(msg)
            return

        crop = color_img[cy1:cy2, cx1:cx2]
        if crop.size == 0:
            msg.has_face = False
            self.pub_crop.publish(msg)
            return

        msg.has_face = True
        try:
            img_msg = self.bridge.cv2_to_imgmsg(crop, encoding='bgr8')
            img_msg.header = msg.header
            msg.face_crop = img_msg
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 转换失败: {e}')
            msg.has_face = False

        self.pub_crop.publish(msg)

    # =========================================================
    # 关闭
    # =========================================================
    def destroy_node(self):
        self.get_logger().info('正在关闭 face_detect_node ...')

        # 1. 停止抓帧线程
        self._stop.set()

        # 2. 取消推理定时器（防止 shutdown 期间回调再触发）
        self.destroy_timer(self.timer)

        # 3. 先停 pipeline（让 wait_for_frames 立即返回）
        try:
            self.pipeline.stop()
        except BaseException:
            pass

        # 4. 等待抓帧线程退出
        try:
            self._grab_thread.join(timeout=2.0)
        except Exception:
            pass

        # 5. 释放检测器
        try:
            self.detector.release()
        except Exception:
            pass

        # 6. ROS 资源清理（放到最后，保证日志通道还活着）
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