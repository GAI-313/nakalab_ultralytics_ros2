"""推論結果を元画像の画素座標で扱う，ROS ノードに依存しない変換処理."""

import math

import numpy as np


def cpu_array(value):
    """Tensor または配列を CPU の NumPy 配列へ変換する."""
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def instances(result, height, width):
    """元画像サイズのマスクを検証し，矩形・クラス・切り出しマスクを返す."""
    if result is None or result.boxes is None or len(result.boxes) == 0:
        return []
    if result.masks is None:
        raise ValueError('検出結果にセグメンテーションマスクがありません．')
    boxes = cpu_array(result.boxes.xyxy)
    confidence = cpu_array(result.boxes.conf)
    classes = cpu_array(result.boxes.cls)
    masks = cpu_array(result.masks.data)
    if (boxes.shape != (len(boxes), 4) or confidence.shape != (len(boxes),)
            or classes.shape != (len(boxes),) or masks.shape != (len(boxes), height, width)):
        raise ValueError('マスク・矩形の個数または元画像サイズが一致しません．')
    output = []
    for index, (box, score, cls, mask) in enumerate(zip(boxes, confidence, classes, masks)):
        if (not np.isfinite(box).all() or not math.isfinite(float(score))
                or not 0 <= score <= 1 or not math.isfinite(float(cls))
                or cls < 0 or cls != int(cls) or not np.isfinite(mask).all()):
            raise ValueError('推論結果に不正な数値があります．')
        left, top = max(0, math.floor(box[0])), max(0, math.floor(box[1]))
        right, bottom = min(width, math.ceil(box[2])), min(height, math.ceil(box[3]))
        if right <= left or bottom <= top:
            continue
        cropped = np.ascontiguousarray((mask[top:bottom, left:right] > 0.5).astype('uint8') * 255)
        if not cropped.any():
            continue
        output.append((index, int(cls), str(result.names[int(cls)]), float(score),
                       (left, top, right, bottom), cropped))
    return output
