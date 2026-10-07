"""モデルの開始・停止と ROS メッセージの構成を検証する."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import nakalab_ultralytics_ros2.object_seg_pose as module
from nakalab_ultralytics_ros2.object_seg_pose import ObjectSegPose
import numpy as np
import pytest
import rclpy


@pytest.fixture
def detector(tmp_path):
    """ダウンロードも GPU も使わないモデルを注入する."""
    rclpy.init(args=[])
    boxes = MagicMock()
    boxes.__len__.return_value = 1
    boxes.xyxy = np.array([[1, 1, 4, 3]])
    boxes.cls, boxes.conf = np.array([41]), np.array([0.8])
    result = SimpleNamespace(boxes=boxes, names={41: 'cup'},
                             masks=SimpleNamespace(data=np.ones((1, 4, 6))))
    model = MagicMock(task='segment', names={41: 'cup'})
    model.predict.return_value = [result]
    factory = MagicMock(return_value=model)
    node = ObjectSegPose(model_factory=factory)
    path = tmp_path / 'model.pt'
    path.touch()
    node.settings.update(model_path=str(path), publish_image=False, class_names=['cup'])
    node.detections = MagicMock()
    try:
        yield node, model, factory
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_start_stop_and_latest_image(detector):
    """最新画像だけを処理し，開始更新・停止要求を反映する."""
    node, model, factory = detector
    assert node.start(0.6)[0]
    image = node.bridge.cv2_to_imgmsg(np.zeros((4, 6, 3), dtype=np.uint8), 'bgr8')
    image.header.frame_id = 'camera'
    image.header.stamp.sec = 10
    node.on_image(image)
    latest = node.bridge.cv2_to_imgmsg(np.zeros((4, 6, 3), dtype=np.uint8), 'bgr8')
    latest.header.frame_id = 'camera'
    latest.header.stamp.sec = 11
    node.on_image(latest)
    node.process_image()
    output = node.detections.publish.call_args.args[0]
    assert output.header == latest.header
    assert (output.image_height, output.image_width) == (4, 6)
    obj, = output.objects
    assert obj.class_name == 'cup'
    assert obj.roi.x_offset == obj.roi.y_offset == 1
    assert (obj.mask.height, obj.mask.width) == (2, 3)
    assert obj.mask.header == latest.header
    assert model.predict.call_args.kwargs['retina_masks'] is True
    assert model.predict.call_args.kwargs['classes'] == [41]
    node.process_image()
    assert model.predict.call_count == 1
    assert node.start(0.7)[0]
    assert factory.call_count == 1
    assert node.confidence == 0.7
    assert node.stop()[0]
    assert node.model is None
    assert node.detections.publish.call_args.args[0].objects == []
    node.on_image(latest)
    assert node.latest_image is None


def test_rejects_wrong_model_and_unknown_class(detector):
    """人物姿勢モデルや不明なクラスを検出に使用しない."""
    node, model, _ = detector
    model.task = 'pose'
    assert not node.start(0.5)[0]
    model.task = 'segment'
    node.settings['class_names'] = ['unknown']
    assert not node.start(0.5)[0]
    assert node.model is None


def test_missing_model_and_invalid_confidence_do_not_load(detector):
    """入力検証に失敗した場合はモデルをロードしない."""
    node, _, factory = detector
    assert not node.start(float('nan'))[0]
    node.settings['model_path'] += '.missing'
    assert not node.start(0.5)[0]
    factory.assert_not_called()


def test_inference_error_publishes_empty_and_recovers(detector):
    """一フレームの失敗後も次のフレームを処理できる."""
    node, model, _ = detector
    assert node.start(0.5)[0]
    image = node.bridge.cv2_to_imgmsg(np.zeros((4, 6, 3), dtype=np.uint8), 'bgr8')
    model.predict.side_effect = RuntimeError('検証用エラー')
    node.on_image(image)
    node.process_image()
    assert node.detections.publish.call_args.args[0].objects == []
    model.predict.side_effect = None
    node.on_image(image)
    node.process_image()
    assert len(node.detections.publish.call_args.args[0].objects) == 1


@pytest.mark.parametrize('context_active', [False, True])
def test_shutdown_race_does_not_hide_runtime_errors(monkeypatch, context_active):
    """終了済み context の競合だけを許容し，通常動作中の例外は伝える."""
    node = MagicMock()
    monkeypatch.setattr(module, 'ObjectSegPose', MagicMock(return_value=node))
    monkeypatch.setattr(rclpy, 'init', MagicMock())
    monkeypatch.setattr(rclpy, 'shutdown', MagicMock())
    monkeypatch.setattr(rclpy, 'ok', lambda: context_active)
    monkeypatch.setattr(rclpy, 'spin', MagicMock(side_effect=RuntimeError('購読処理失敗')))
    if context_active:
        with pytest.raises(RuntimeError):
            module.main([])
        rclpy.shutdown.assert_called_once()
    else:
        module.main([])
        rclpy.shutdown.assert_not_called()
    node.destroy_node.assert_called_once()
