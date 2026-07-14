#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
可视化节点
- 订阅 /face/crop (face_msgs/FaceCrop) 显示裁剪人脸图
- 订阅 /face/target (face_msgs/FaceTarget) 在终端打印 3D 坐标
- 按 q 或 ESC 退出
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

from face_msgs.msg import FaceCrop, FaceTarget


class FaceVizNode(Node):
    def __init__(self):
        super().__init__('face_viz_node')

        self.declare_parameter('crop_topic',   '/face/crop')
        self.declare_parameter('target_topic', '/face/target')
        self.declare_parameter('window_name',  'Face Crop (q=quit)')

        self.win_name = self.get_parameter('window_name').value
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.sub_crop = self.create_subscription(
            FaceCrop,
            self.get_parameter('crop_topic').value,
            self.on_crop,
            qos,
        )

        self.sub_target = self.create_subscription(
            FaceTarget,
            self.get_parameter('target_topic').value,
            self.on_target,
            qos,
        )

        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f'订阅: {self.get_parameter("crop_topic").value}')
        self.get_logger().info(f'订阅: {self.get_parameter("target_topic").value}')
        self.get_logger().info('按 q 或 ESC 退出')

        self._target_seq = 0  # 节流 target 打印

    def on_crop(self, msg: FaceCrop):
        if not msg.has_face:
            # 没人脸时显示空白提示
            blank = 480 * [640 * [0]]  # dummy
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg.face_crop, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 转换失败: {e}')
            return

        cv2.imshow(self.win_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('用户退出')
            rclpy.shutdown()

    def on_target(self, msg: FaceTarget):
        # 每 30 帧打印一次，避免刷屏
        self._target_seq += 1
        if self._target_seq % 30 != 0:
            return
        if msg.has_target:
            self.get_logger().info(
                f'🎯 人脸 3D: X={msg.center.x:.3f} Y={msg.center.y:.3f} Z={msg.center.z:.3f} m'
            )
        else:
            self.get_logger().info('🎯 无人脸')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceVizNode()
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
