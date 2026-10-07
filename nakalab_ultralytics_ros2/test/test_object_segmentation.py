"""元画像に対応するマスクと矩形の変換を検証する."""

from types import SimpleNamespace

from nakalab_ultralytics_ros2.object_segmentation import instances
import numpy as np
import pytest


class Boxes:
    """モデル出力と同じ配列の境界条件を用意する."""

    def __init__(self, xyxy, conf=(0.9,), cls=(41,)):
        """矩形と付随する数値を配列として保持する."""
        self.xyxy = np.array(xyxy, dtype=float)
        self.conf = np.array(conf, dtype=float)
        self.cls = np.array(cls, dtype=float)

    def __len__(self):
        """検出数を返す."""
        return len(self.xyxy)


def result():
    """穴と離れた領域を含むマスクを作る."""
    mask = np.ones((1, 5, 7), dtype=np.uint8)
    mask[0, 2, 3] = 0
    return SimpleNamespace(boxes=Boxes([[-1.2, 1.2, 6.8, 8.1]]),
                           masks=SimpleNamespace(data=mask), names={41: 'cup'})


def test_original_pixels_crop_and_hole():
    """マスクの穴と元画像の座標が切り出し後も保存される."""
    item, = instances(result(), 5, 7)
    assert item[:3] == (0, 41, 'cup')
    assert item[4] == (0, 1, 7, 5)
    assert item[5].shape == (4, 7)
    assert item[5][1, 3] == 0
    assert item[5][0, 0] == 255
    assert item[5].flags.c_contiguous


@pytest.mark.parametrize('shape', [(1, 3, 4), (2, 5, 7)])
def test_rejects_scaled_or_mismatched_masks(shape):
    """縮小画像のマスクを元画像として誤用しない."""
    data = result()
    data.masks.data = np.ones(shape)
    with pytest.raises(ValueError):
        instances(data, 5, 7)


@pytest.mark.parametrize('field,value', [('conf', np.nan), ('conf', 1.1),
                                         ('cls', -1), ('cls', 1.5)])
def test_rejects_invalid_numbers(field, value):
    """信頼度とクラスの不正値を拒否する."""
    data = result()
    getattr(data.boxes, field)[0] = value
    with pytest.raises(ValueError):
        instances(data, 5, 7)


def test_missing_masks_and_empty_detections():
    """検出ゼロとセグメンテーション欠落を区別する."""
    assert instances(None, 5, 7) == []
    data = result()
    data.masks = None
    with pytest.raises(ValueError):
        instances(data, 5, 7)
    data.boxes = Boxes([], (), ())
    assert instances(data, 5, 7) == []


def test_empty_or_outside_region_is_skipped():
    """有効画素を持たない領域は出力しない."""
    data = result()
    data.masks.data[:] = 0
    assert instances(data, 5, 7) == []
    data.masks.data[:] = 1
    data.boxes.xyxy[:] = [10, 10, 12, 12]
    assert instances(data, 5, 7) == []
