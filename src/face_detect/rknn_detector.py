#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8-Face RKNN 推理类（从 detect.py 提炼，逻辑完全一致）
- 不依赖 ROS，可独立测试
- 输入: BGR 原图 (np.ndarray, HxWx3, uint8)
- 输出: boxes (N,4) xyxy 原图坐标, scores (N,)
"""

import cv2
import numpy as np
from rknnlite.api import RKNNLite


class RKNNFaceDetector:
    def __init__(self,
                 model_path: str,
                 img_size: int = 640,
                 conf_thr: float = 0.25,
                 iou_thr: float = 0.45,
                 core_mask=None):
        self.img_size = img_size
        self.conf_thr = conf_thr
        self.iou_thr  = iou_thr

        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f'load_rknn 失败: {model_path}')

        if core_mask is None:
            core_mask = RKNNLite.NPU_CORE_0
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError('init_runtime 失败')

    # ---------------- 预处理 ----------------
    @staticmethod
    def letterbox(im, new_shape=640, color=(114, 114, 114)):
        h, w = im.shape[:2]
        r = min(new_shape / h, new_shape / w)
        new_w, new_h = int(round(w * r)), int(round(h * r))
        dw = (new_shape - new_w) // 2
        dh = (new_shape - new_h) // 2
        im_resized = cv2.resize(im, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        im_padded = cv2.copyMakeBorder(
            im_resized, dh, new_shape - new_h - dh,
            dw, new_shape - new_w - dw,
            cv2.BORDER_CONSTANT, value=color
        )
        return im_padded, r, dw, dh

    # ---------------- NMS ----------------
    @staticmethod
    def nms(boxes, scores, iou_thr):
        x1, y1, x2, y2 = boxes[:,0], boxes[:,1], boxes[:,2], boxes[:,3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            w = np.maximum(0., xx2 - xx1)
            h = np.maximum(0., yy2 - yy1)
            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
            order = order[1:][iou < iou_thr]
        return keep

    # ---------------- 后处理 ----------------
    def post_process(self, output):
        pred = output[0]              # (5, 8400)
        pred = pred.transpose(1, 0)   # (8400, 5)

        scores = pred[:, 4]
        mask = scores > self.conf_thr
        if mask.sum() == 0:
            return np.empty((0,4), dtype=np.float32), np.empty((0,), dtype=np.float32)

        pred = pred[mask]
        scores = scores[mask]

        cx, cy, w, h = pred[:,0], pred[:,1], pred[:,2], pred[:,3]
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        boxes = np.stack([x1, y1, x2, y2], axis=-1)

        keep = self.nms(boxes, scores, self.iou_thr)
        return boxes[keep], scores[keep]

    @staticmethod
    def scale_back(boxes, r, dw, dh, orig_w, orig_h):
        boxes = boxes.copy()
        boxes[:, [0,2]] = (boxes[:, [0,2]] - dw) / r
        boxes[:, [1,3]] = (boxes[:, [1,3]] - dh) / r
        boxes[:, [0,2]] = np.clip(boxes[:, [0,2]], 0, orig_w - 1)
        boxes[:, [1,3]] = np.clip(boxes[:, [1,3]], 0, orig_h - 1)
        return boxes

    # ---------------- 对外主接口 ----------------
    def detect(self, img_bgr):
        """
        输入: BGR 原图 (H, W, 3) uint8
        输出: boxes (N,4) xyxy 原图坐标 int, scores (N,) float
        """
        H0, W0 = img_bgr.shape[:2]

        # 预处理
        img, r, dw, dh = self.letterbox(img_bgr, self.img_size)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(img_rgb, 0)  # (1,640,640,3) uint8

        # 推理
        outputs = self.rknn.inference(inputs=[inp], data_format='nhwc')

        # 后处理
        boxes, scores = self.post_process(outputs[0])
        boxes = self.scale_back(boxes, r, dw, dh, W0, H0)
        return boxes.astype(np.int32), scores.astype(np.float32)

    def release(self):
        try:
            self.rknn.release()
        except Exception:
            pass