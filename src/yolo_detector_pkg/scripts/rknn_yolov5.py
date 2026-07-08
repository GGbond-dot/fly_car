#!/usr/bin/env python3
"""YOLOv5 + RKNN(RK3588 NPU)推理与后处理,供 ROS2 节点和独立测试脚本共用。

模型: yoloqian_formal_best_rk3588_int8.rknn (YOLOv5s, 640x640, 2 类 rescuee1/rescuee2)。

后处理同时兼容两种常见 RKNN 导出:
  A) rknn_model_zoo 标准: 3 个输出分支 [1, 3*(5+nc), h, w] (h/w=80/40/20),
     需 sigmoid + anchor 解码 (anchors 为 yolov5 默认)。
  B) 单输出 [1, N, 5+nc]: 已在图内解码,只做阈值 + NMS。
运行时按实际输出个数/维度自动选择,并打印一次实际 shape 便于核对。
"""

import numpy as np

# YOLOv5 默认 anchors(与 yolov5s.yaml 一致),按 stride 8/16/32 分三组
DEFAULT_ANCHORS = [
    [10, 13, 16, 30, 33, 23],       # P3/8
    [30, 61, 62, 45, 59, 119],      # P4/16
    [116, 90, 156, 198, 373, 326],  # P5/32
]
DEFAULT_STRIDES = [8, 16, 32]


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def letterbox(img, new_shape=640, color=(114, 114, 114)):
    """按 YOLOv5 方式等比缩放 + 灰边填充到 new_shape,返回 (图, 缩放比, (dw, dh))。"""
    import cv2
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw = (new_shape[1] - new_unpad[0]) / 2
    dh = (new_shape[0] - new_unpad[1]) / 2
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=color)
    return img, r, (left, top)


def _decode_branch(out, anchors, stride, img_size, nc):
    """分支输出 [1, 3*(5+nc), h, w] -> (boxes_xyxy, obj_conf, cls_probs),像素坐标(640系)。"""
    na = 3
    _, _, gh, gw = out.shape
    # -> [h, w, 3, 5+nc]
    x = out.reshape(na, 5 + nc, gh, gw).transpose(2, 3, 0, 1)

    box_xy = sigmoid(x[..., 0:2]) * 2.0 - 0.5
    box_wh = (sigmoid(x[..., 2:4]) * 2.0) ** 2
    obj = sigmoid(x[..., 4:5])
    cls = sigmoid(x[..., 5:])

    col = np.arange(gw).reshape(1, gw, 1)
    row = np.arange(gh).reshape(gh, 1, 1)
    grid_x = np.broadcast_to(col, (gh, gw, na))
    grid_y = np.broadcast_to(row, (gh, gw, na))
    grid = np.stack((grid_x, grid_y), axis=-1)  # [h,w,3,2]

    box_xy = (box_xy + grid) * stride
    anchor = np.array(anchors, dtype=np.float32).reshape(na, 2)
    box_wh = box_wh * anchor

    boxes = np.concatenate((box_xy, box_wh), axis=-1).reshape(-1, 4)
    obj = obj.reshape(-1, 1)
    cls = cls.reshape(-1, nc)
    # xywh -> xyxy
    xyxy = np.empty_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return xyxy, obj, cls


def _nms(boxes, scores, iou_thresh):
    """标准 NMS,boxes 为 xyxy。返回保留下标列表。"""
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thresh]
    return keep


def postprocess(outputs, nc, img_size=640, conf_thresh=0.25, nms_thresh=0.45,
                anchors=DEFAULT_ANCHORS, strides=DEFAULT_STRIDES):
    """把 RKNN 原始输出解成 [ (x1,y1,x2,y2,conf,cls), ... ](640 letterbox 像素系)。"""
    outs = [np.asarray(o) for o in outputs]

    all_boxes, all_conf, all_cls = [], [], []
    if len(outs) >= 3:
        # A) 3 分支,需 anchor 解码。按每个分支的 grid 尺寸自动推 stride 并匹配 anchor,
        #    不假设输出顺序(不同导出 80/40/20 的排列可能不同)。
        stride_to_anchor = dict(zip(strides, anchors))
        for out in outs[:3]:
            if out.ndim == 4 and out.shape[1] == 3 * (5 + nc):
                pass
            elif out.ndim == 4 and out.shape[-1] == 3 * (5 + nc):
                out = out.transpose(0, 3, 1, 2)  # NHWC -> NCHW
            gh = out.shape[2]
            stride = int(round(img_size / gh))
            anchor = stride_to_anchor.get(stride, anchors[0])
            xyxy, obj, cls = _decode_branch(out, anchor, stride, img_size, nc)
            all_boxes.append(xyxy)
            all_conf.append(obj)
            all_cls.append(cls)
        boxes = np.concatenate(all_boxes, 0)
        obj = np.concatenate(all_conf, 0)
        cls = np.concatenate(all_cls, 0)
        cls_id = cls.argmax(1)
        cls_score = cls.max(1)
        scores = obj[:, 0] * cls_score
    else:
        # B) 单输出 [1, N, 5+nc],已解码(xywh) -> xyxy
        pred = outs[0]
        if pred.ndim == 3:
            pred = pred[0]
        xywh = pred[:, :4]
        obj = pred[:, 4]
        cls = pred[:, 5:5 + nc]
        cls_id = cls.argmax(1)
        cls_score = cls.max(1)
        scores = obj * cls_score
        boxes = np.empty((xywh.shape[0], 4), dtype=np.float32)
        boxes[:, 0] = xywh[:, 0] - xywh[:, 2] / 2
        boxes[:, 1] = xywh[:, 1] - xywh[:, 3] / 2
        boxes[:, 2] = xywh[:, 0] + xywh[:, 2] / 2
        boxes[:, 3] = xywh[:, 1] + xywh[:, 3] / 2

    mask = scores >= conf_thresh
    boxes, scores, cls_id = boxes[mask], scores[mask], cls_id[mask]
    if boxes.shape[0] == 0:
        return []

    # 逐类 NMS
    results = []
    for c in np.unique(cls_id):
        idx = np.where(cls_id == c)[0]
        keep = _nms(boxes[idx], scores[idx], nms_thresh)
        for k in keep:
            j = idx[k]
            x1, y1, x2, y2 = boxes[j]
            results.append((float(x1), float(y1), float(x2), float(y2),
                            float(scores[j]), int(c)))
    return results


def scale_boxes(dets, ratio, pad, orig_w, orig_h):
    """把 640 letterbox 系的框映射回原始帧像素系,并裁剪到画面内。"""
    dx, dy = pad
    out = []
    for x1, y1, x2, y2, conf, cls in dets:
        x1 = (x1 - dx) / ratio
        y1 = (y1 - dy) / ratio
        x2 = (x2 - dx) / ratio
        y2 = (y2 - dy) / ratio
        x1 = max(0.0, min(orig_w - 1.0, x1))
        y1 = max(0.0, min(orig_h - 1.0, y1))
        x2 = max(0.0, min(orig_w - 1.0, x2))
        y2 = max(0.0, min(orig_h - 1.0, y2))
        out.append((x1, y1, x2, y2, conf, cls))
    return out


class RknnYolov5:
    """封装 RKNNLite 加载 + 推理 + 后处理。"""

    def __init__(self, model_path, class_names, img_size=640,
                 conf_thresh=0.25, nms_thresh=0.45,
                 anchors=DEFAULT_ANCHORS, strides=DEFAULT_STRIDES,
                 core_mask=None, logger=None):
        from rknnlite.api import RKNNLite
        self.class_names = class_names
        self.nc = len(class_names)
        self.img_size = img_size
        self.conf_thresh = conf_thresh
        self.nms_thresh = nms_thresh
        self.anchors = anchors
        self.strides = strides
        self._log = logger or (lambda m: print(m))
        self._shape_logged = False

        self.rknn = RKNNLite()
        if self.rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"load_rknn failed: {model_path}")
        # RK3588 多核 NPU;core_mask 可指定 NPU_CORE_0/AUTO 等,默认 AUTO
        if core_mask is None:
            core_mask = RKNNLite.NPU_CORE_AUTO
        if self.rknn.init_runtime(core_mask=core_mask) != 0:
            raise RuntimeError("init_runtime failed")
        self._log(f"RKNN loaded: {model_path} (nc={self.nc}, imgsz={img_size})")

    def infer(self, frame_bgr):
        """输入 BGR 原始帧,返回 [(x1,y1,x2,y2,conf,cls_id), ...](原始帧像素系)。"""
        import cv2
        h0, w0 = frame_bgr.shape[:2]
        img, ratio, pad = letterbox(frame_bgr, self.img_size)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        outputs = self.rknn.inference(inputs=[img_rgb], data_format="nhwc")
        if not self._shape_logged:
            self._log("RKNN output shapes: " +
                      ", ".join(str(np.asarray(o).shape) for o in outputs))
            self._shape_logged = True
        dets = postprocess(outputs, self.nc, self.img_size,
                           self.conf_thresh, self.nms_thresh,
                           self.anchors, self.strides)
        return scale_boxes(dets, ratio, pad, w0, h0)

    def release(self):
        try:
            self.rknn.release()
        except Exception:
            pass


def draw_detections(frame_bgr, dets, class_names):
    """在帧上画框+类名+置信度(BGR,原地返回同一帧)。"""
    import cv2
    colors = [(0, 200, 0), (0, 128, 255), (255, 0, 0), (255, 0, 255)]
    for x1, y1, x2, y2, conf, cls in dets:
        color = colors[cls % len(colors)]
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(frame_bgr, p1, p2, color, 2)
        name = class_names[cls] if 0 <= cls < len(class_names) else str(cls)
        label = f"{name} {conf:.2f}"
        cv2.putText(frame_bgr, label, (int(x1), max(0, int(y1) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return frame_bgr


def load_class_names(path):
    with open(path, "r", encoding="utf-8") as f:
        return [ln.strip() for ln in f if ln.strip()]
