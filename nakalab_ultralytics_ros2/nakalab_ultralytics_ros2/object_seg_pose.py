#!/usr/bin/env python3
"""物体ごとの YOLO セグメンテーションを配信するノード."""

import gc
import math
import os
from pathlib import Path

from cv_bridge import CvBridge
from nakalab_ultralytics_interfaces.msg import ObjectSeg2D, ObjectSeg2DArray
from nakalab_ultralytics_interfaces.srv import Detect
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .object_segmentation import instances


class ObjectSegPose(Node):
    """最新の RGB 画像を処理し，サービスでモデルの開始・停止を制御する."""

    def __init__(self, model_factory=None):
        """パラメータと通信資源を初期化する."""
        super().__init__('object_seg_pose')
        defaults = {
            'model_path': '/tmp/yolo26l-seg.pt', 'device': 'cuda',
            'confidence': 0.5, 'imgsz': 640, 'max_det': 20,
            'class_names': ['*'], 'max_rate_hz': 10.0, 'run_detect': False,
            'publish_image': True,
        }
        values = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.settings = values
        self.confidence = float(values['confidence'])
        if (not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1
                or not math.isfinite(values['max_rate_hz']) or values['max_rate_hz'] <= 0
                or values['imgsz'] <= 0 or values['max_det'] <= 0):
            raise ValueError('信頼度・画像サイズ・最大検出数・処理周期を確認してください．')
        self.bridge = CvBridge()
        self.model_factory = model_factory
        self.model = None
        self.class_ids = None
        self.latest_image = None
        self.last_header = None
        self.detections = self.create_publisher(ObjectSeg2DArray, 'nu_ros2/object_seg_2d', 10)
        self.annotated = self.create_publisher(
            Image, 'nu_ros2/object_detect_image', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, 'color_image', self.on_image, qos_profile_sensor_data)
        self.service = self.create_service(Detect, 'nu_ros2/detect_object', self.on_detect)
        # 単一 executor で推論と開始・停止を直列化し，待ち画像は最新の一枚に限定する．
        self.timer = self.create_timer(1.0 / values['max_rate_hz'], self.process_image)
        if values['run_detect']:
            success, message = self.start(self.confidence)
            if not success:
                self.destroy_node()
                raise RuntimeError(message)

    def start(self, confidence):
        """ローカルの seg モデルを読み込み，検出を開始する."""
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            return False, 'confidence は 0 以上 1 以下の有限値を指定してください．'
        if self.model is not None:
            self.confidence = confidence
            return True, '検出中の信頼度を更新しました．'
        path = Path(self.settings['model_path']).expanduser()
        if not path.is_file():
            return False, f'ローカルモデルがありません: {path}'
        try:
            factory = self.model_factory
            if factory is None:
                # 重みの自動ダウンロードを行わず，配置済みの資産だけを使う．
                os.environ['YOLO_OFFLINE'] = 'true'
                from ultralytics import YOLO
                factory = YOLO
            candidate = factory(str(path))
            if candidate.task != 'segment':
                raise ValueError('segment タスクのモデルが必要です．')
            names = candidate.names
            requested = self.settings['class_names']
            if requested != ['*']:
                unknown = set(requested) - set(names.values())
                if not requested or unknown:
                    raise ValueError(f'モデルにない対象クラスです: {sorted(unknown)}')
                self.class_ids = [int(key) for key, value in names.items() if value in requested]
            self.model, self.confidence = candidate, confidence
            self.latest_image = None
            self.get_logger().info(f'物体検出を開始しました: {path}')
            return True, '物体検出を開始しました．'
        except Exception as exc:
            self.model = None
            self.get_logger().error(f'モデルを読み込めません: {exc}')
            return False, str(exc)

    def stop(self):
        """待ち画像とモデルを解放し，空の検出結果を通知する."""
        self.model = None
        self.latest_image = None
        self.class_ids = None
        if rclpy.ok(context=self.context):
            self.publish_empty(self.last_header)
        gc.collect()
        # CPU 検証時に torch を新規ロードしない．
        import sys
        torch = sys.modules.get('torch')
        if torch is not None and torch.cuda.is_available():
            torch.cuda.empty_cache()
        return True, '物体検出を停止しました．'

    def on_detect(self, request, response):
        """既存 Detect.srv と同じ開始・停止要求を受け付ける."""
        response.success, response.message = (
            self.start(float(request.confidence)) if request.run else self.stop())
        return response

    def on_image(self, message):
        """待ち画像を置き換え，推論遅延による無制限な蓄積を防ぐ."""
        if self.model is not None:
            self.latest_image = message

    def publish_empty(self, header):
        """検出ゼロと停止を，空配列として通知する."""
        message = ObjectSeg2DArray()
        if header is not None:
            message.header = header
        self.detections.publish(message)

    def process_image(self):
        """最新画像を推論し，元画像と同じ時刻・座標系で結果を配信する."""
        if self.model is None or self.latest_image is None:
            return
        source, self.latest_image = self.latest_image, None
        self.last_header = source.header
        try:
            image = self.bridge.imgmsg_to_cv2(source, desired_encoding='bgr8')
            results = self.model.predict(
                source=image, device=self.settings['device'], conf=self.confidence,
                imgsz=self.settings['imgsz'], max_det=self.settings['max_det'],
                classes=self.class_ids, retina_masks=True, verbose=False, save=False)
            result = results[0] if results else None
            output = ObjectSeg2DArray()
            output.header = source.header
            output.image_height, output.image_width = image.shape[:2]
            for index, cls, name, score, box, mask in instances(result, *image.shape[:2]):
                obj = ObjectSeg2D(
                    detection_id=index, class_id=cls, class_name=name, confidence=score)
                left, top, right, bottom = box
                obj.bounding_box.top_left.x, obj.bounding_box.top_left.y = float(left), float(top)
                obj.bounding_box.bottom_right.x = float(right)
                obj.bounding_box.bottom_right.y = float(bottom)
                obj.bounding_box.confidence = score
                obj.roi.x_offset, obj.roi.y_offset = left, top
                obj.roi.width, obj.roi.height = right - left, bottom - top
                obj.mask = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                obj.mask.header = source.header
                output.objects.append(obj)
            self.detections.publish(output)
            if self.settings['publish_image']:
                painted = result.plot() if result is not None else image
                message = self.bridge.cv2_to_imgmsg(painted, encoding='bgr8')
                message.header = source.header
                self.annotated.publish(message)
        except Exception as exc:
            self.get_logger().error(f'物体検出フレームを処理できません: {exc}')
            self.publish_empty(source.header)

    def destroy_node(self):
        """タイマーと推論資源を停止する."""
        self.timer.cancel()
        self.stop()
        return super().destroy_node()


def main(args=None):
    """ROS ノードを起動し，終了時に資源を解放する."""
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectSegPose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # Humble の終了シグナルと購読処理の競合は，context 終了時だけ許容する．
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
