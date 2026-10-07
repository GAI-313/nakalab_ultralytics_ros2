"""古い検出の除外，停止保証，失敗時の API 動作を検証する."""

from unittest.mock import MagicMock

from nakalab_ultralytics_api.object_detector import ObjectDetector
from nakalab_ultralytics_interfaces.msg import ObjectSeg3D, ObjectSeg3DArray
import pytest


@pytest.fixture
def client(monkeypatch):
    """壁時計と ROS 時刻の境界を固定したクライアントを用意する."""
    node = MagicMock()
    node.get_clock().now().nanoseconds = 10_000_000_000
    detector = ObjectDetector(node)
    detector.execute = MagicMock(return_value=True)
    monkeypatch.setattr('rclpy.ok', lambda: True)
    return detector


def test_selects_fresh_valid_objects_and_stops(client, monkeypatch):
    """古い結果・無効位置・対象外クラスを除外する."""
    old = ObjectSeg3DArray()
    old.header.stamp.sec = 9
    old.objects = [ObjectSeg3D(class_name='cup', valid=True)]
    new = ObjectSeg3DArray()
    new.header.stamp.sec = 11
    new.objects = [ObjectSeg3D(class_name='cup', valid=False),
                   ObjectSeg3D(class_name='bottle', valid=True),
                   ObjectSeg3D(class_name='cup', valid=True)]
    messages = iter([old, new])
    monkeypatch.setattr('rclpy.spin_once', lambda *a, **kw: client._on_3d(next(messages)))
    result = client.detect(class_names=['cup'])
    assert result.header.stamp.sec == 11
    assert len(result.objects) == 1
    result.objects[0].class_name = 'changed'
    assert new.objects[2].class_name == 'cup'
    assert [call.args[0] for call in client.execute.call_args_list] == [True, False]


def test_timeout_stops_model(client, monkeypatch):
    """検出待ちが時間切れでも停止要求を送る."""
    monkeypatch.setattr('rclpy.spin_once', lambda *a, **kw: None)
    assert client.detect(timeout_sec=0.01) is None
    assert [call.args[0] for call in client.execute.call_args_list] == [True, False]


def test_processing_exception_still_stops(client, monkeypatch):
    """例外発生時にも起動した検出を停止する."""
    monkeypatch.setattr('rclpy.spin_once', MagicMock(side_effect=RuntimeError('通信失敗')))
    with pytest.raises(RuntimeError):
        client.detect()
    assert client.execute.call_args.args[0] is False


def test_start_failure_is_explicit(client):
    """検出開始失敗を例外として呼び出し側へ通知する."""
    client.execute.return_value = False
    with pytest.raises(RuntimeError):
        client.detect()
    assert client.execute.call_count == 1


def test_invalid_arguments_do_not_start(client):
    """不正な要求は検出を開始する前に拒否する."""
    with pytest.raises(ValueError):
        client.detect(timeout_sec=float('nan'))
    with pytest.raises(ValueError):
        client.detect(class_names='cup')
    client.execute.assert_not_called()
