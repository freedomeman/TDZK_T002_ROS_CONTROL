#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OpenCV Haar Cascade 人脸检测器
- 不依赖 ROS，不依赖外部模型下载，可独立测试
- 输入: BGR 原图 (np.ndarray, HxWx3, uint8)
- 输出: boxes (N,4) xyxy 原图坐标 int32, scores (N,) float32
"""

import os
import cv2
import numpy as np

# Haar cascade 文件路径（自动从同目录查找，或使用缓存）
_CASCADE_PATH = os.path.join(os.path.dirname(__file__), 'haarcascade_frontalface_default.xml')
_CACHE_PATH = os.path.expanduser('~/.cache/opencv/haarcascade_frontalface_default.xml')


class FaceDetector:
    def __init__(self, conf_thr: float = 0.5, **_kwargs):
        self.conf_thr = conf_thr

        # 查找 cascade 文件
        cascade_path = None
        for p in (_CASCADE_PATH, _CACHE_PATH):
            if os.path.exists(p):
                cascade_path = p
                break

        if cascade_path is None:
            # 自动下载到缓存目录
            os.makedirs(os.path.dirname(_CACHE_PATH), exist_ok=True)
            import urllib.request
            url = 'https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml'
            urllib.request.urlretrieve(url, _CACHE_PATH)
            cascade_path = _CACHE_PATH

        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f'无法加载 Haar cascade: {cascade_path}')

    def detect(self, img_bgr):
        """
        输入: BGR 原图 (H, W, 3) uint8
        输出: boxes (N,4) xyxy 原图坐标 int32, scores (N,) float32
        """
        H, W = img_bgr.shape[:2]
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        # 参数: scaleFactor=1.1, minNeighbors=4, minSize=(30,30)
        rects = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=4,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE,
        )

        boxes = []
        scores = []
        for (x, y, w, h) in rects:
            x1 = max(0, x)
            y1 = max(0, y)
            x2 = min(W, x + w)
            y2 = min(H, y + h)
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2, y2])
            scores.append(1.0)  # Haar 无置信度，统一为 1.0

        if boxes:
            return np.array(boxes, dtype=np.int32), np.array(scores, dtype=np.float32)
        return np.empty((0, 4), dtype=np.int32), np.empty((0,), dtype=np.float32)

    def release(self):
        pass
