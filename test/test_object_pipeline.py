"""インストール済みノード間の通信と，配置済みモデルの結合試験."""

from contextlib import contextmanager
import copy
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from nakalab_ultralytics_api.object_detector import ObjectDetector
from nakalab_ultralytics_interfaces.msg import ObjectSeg2D, ObjectSeg2DArray, ObjectSeg3DArray
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


@contextmanager
def process(package, executable, *parameters):
    """起動したプロセス群だけを追跡し，必ず終了を待つ."""
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    command = [str(path), '--ros-args']
    for parameter in parameters:
        command.extend(['-p', parameter])
    child = subprocess.Popen(command, start_new_session=True)
    print(f'node_pid={child.pid} stop_command="kill -INT -- -{child.pid}"', flush=True)
    try:
        yield child
        assert child.poll() is None, f'{executable} が予期せず終了しました．'
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
        try:
            code = child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            code = child.wait(timeout=5)
        print(f'node_exit_pid={child.pid} returncode={code}', flush=True)
        assert code == 0, f'{executable} の終了コード: {code}'


@pytest.fixture
def node():
    """ユーザーの ROS グラフと分離した専用ドメインで通信する."""
    if os.environ.get('ROS_DOMAIN_ID') != '211':
        pytest.skip('ROS_DOMAIN_ID=211 の専用検証環境で実行してください．')
    rclpy.init(args=[])
    instance = Node('object_pipeline_test')
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def spin_until(node, predicate, timeout=10.0):
    """条件成立まで上限時間付きでコールバックを進める."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError('期待した ROS メッセージが時間内に届きませんでした．')


def test_projection_tf_invalid_empty_and_stale(node):
    """座標変換，無効深度，検出消去，入力停止時の消去を検証する."""
    with process('nakalab_ultralytics_cpp', 'object_seg_pose_3d',
                 'ref_frame:=test_base', 'pixel_stride:=1', 'min_points:=1',
                 'stale_timeout_sec:=0.5'):
        received = []
        node.create_subscription(ObjectSeg3DArray, 'nu_ros2/object_seg_3d', received.append, 10)
        objects_pub = node.create_publisher(ObjectSeg2DArray, 'nu_ros2/object_seg_2d', 10)
        depth_pub = node.create_publisher(Image, 'depth_image', qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, 'color_camera_info', qos_profile_sensor_data)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'test_base', 'test_camera'
        tf.transform.translation.x, tf.transform.translation.y = 1.0, 2.0
        tf.transform.translation.z = 3.0
        tf.transform.rotation.z = tf.transform.rotation.w = 2 ** -0.5
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(tf)
        bridge = CvBridge()
        depth = bridge.cv2_to_imgmsg(np.full((3, 4), 2000, dtype=np.uint16), '16UC1')
        info = CameraInfo(width=4, height=3, k=[2., 0., 1., 0., 2., 1., 0., 0., 1.])
        objects = ObjectSeg2DArray(image_width=4, image_height=3)
        obj = ObjectSeg2D(class_name='cup', confidence=0.9)
        obj.roi.x_offset, obj.roi.width, obj.roi.height = 1, 2, 3
        obj.mask = bridge.cv2_to_imgmsg(
            np.array([[255, 255], [255, 0], [255, 255]], dtype=np.uint8), 'mono8')
        objects.objects = [obj]

        def publish():
            objects.header.stamp = node.get_clock().now().to_msg()
            objects.header.frame_id = 'test_camera'
            depth.header = info.header = copy.deepcopy(objects.header)
            for item in objects.objects:
                item.mask.header = copy.deepcopy(objects.header)
            info_pub.publish(info)
            depth_pub.publish(depth)
            objects_pub.publish(objects)

        timer = node.create_timer(0.05, publish)
        spin_until(node, lambda: received and received[-1].objects
                   and received[-1].objects[0].valid)
        result = received[-1]
        assert result.header.frame_id == 'test_base'
        assert result.source_header.frame_id == 'test_camera'
        assert result.header.stamp == result.source_header.stamp
        point = result.objects[0]
        assert [point.centroid.x, point.centroid.y, point.centroid.z] == pytest.approx([1, 2.4, 5])
        assert [point.size.x, point.size.y, point.size.z] == pytest.approx([2, 1, 0], abs=1e-8)
        timer.cancel()
        spin_until(node, lambda: received and not received[-1].objects)

        received.clear()
        timer.reset()
        depth.data = bytes(len(depth.data))
        spin_until(node, lambda: received and received[-1].objects
                   and not received[-1].objects[0].valid)
        assert np.isnan(received[-1].objects[0].centroid.x)
        assert received[-1].objects[0].reason
        objects.objects = []
        spin_until(node, lambda: received and not received[-1].objects)
        timer.cancel()


def test_real_segmentation_and_detector_api(node):
    """ローカルの実モデルと合成深度で RGB から API 取得まで検証する."""
    model = os.environ.get('OBJECT_SEG_TEST_MODEL')
    image_path = os.environ.get('OBJECT_SEG_TEST_IMAGE')
    if not model or not image_path:
        pytest.skip('OBJECT_SEG_TEST_MODEL と OBJECT_SEG_TEST_IMAGE を指定してください．')
    assert Path(model).is_file() and Path(image_path).is_file()
    image = cv2.imread(image_path)
    assert image is not None
    height, width = image.shape[:2]
    device = os.environ.get('OBJECT_SEG_TEST_DEVICE', 'cpu')
    with process('nakalab_ultralytics_cpp', 'object_seg_pose_3d', 'ref_frame:=test_camera'), \
            process('nakalab_ultralytics_ros2', 'object_seg_pose', f'model_path:={model}',
                    f'device:={device}', 'class_names:=[bus]', 'max_rate_hz:=2.0'):
        bridge = CvBridge()
        color = bridge.cv2_to_imgmsg(image, 'bgr8')
        depth = bridge.cv2_to_imgmsg(np.full((height, width), 2000, dtype=np.uint16), '16UC1')
        info = CameraInfo(width=width, height=height,
                          k=[600., 0., width / 2, 0., 600., height / 2, 0., 0., 1.])
        color_pub = node.create_publisher(Image, 'color_image', qos_profile_sensor_data)
        depth_pub = node.create_publisher(Image, 'depth_image', qos_profile_sensor_data)
        info_pub = node.create_publisher(CameraInfo, 'color_camera_info', qos_profile_sensor_data)
        masks, annotations = [], []
        node.create_subscription(ObjectSeg2DArray, 'nu_ros2/object_seg_2d', masks.append, 10)
        node.create_subscription(Image, 'nu_ros2/object_detect_image', annotations.append,
                                 qos_profile_sensor_data)

        def publish():
            color.header.stamp = node.get_clock().now().to_msg()
            color.header.frame_id = 'test_camera'
            depth.header = info.header = copy.deepcopy(color.header)
            info_pub.publish(info)
            depth_pub.publish(depth)
            color_pub.publish(color)

        timer = node.create_timer(0.1, publish)
        detector = ObjectDetector(node, service_timeout_sec=30.0)
        try:
            result = detector.detect(timeout_sec=60.0, class_names=['bus'])
            assert result is not None
            assert all(obj.class_name == 'bus' and obj.valid for obj in result.objects)
            assert all(abs(obj.centroid.z - 2.0) < 1e-6 for obj in result.objects)
            valid_masks = [msg for msg in masks if msg.objects]
            assert valid_masks and annotations
            output = valid_masks[-1]
            assert (output.image_height, output.image_width) == (height, width)
            assert all(obj.mask.header == output.header for obj in output.objects)
            assert any(bridge.imgmsg_to_cv2(obj.mask).any() for obj in output.objects)
            assert (annotations[-1].height, annotations[-1].width) == (height, width)
            spin_until(node, lambda: masks and not masks[-1].objects)
            print(f'real_model_detections={len(result.objects)} depth_m=2.0', flush=True)
        finally:
            timer.cancel()
            detector.close()
