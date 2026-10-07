"""物体検出の開始・停止と，取得時刻を確認した検出結果の受け取り."""

import copy
import math
import time

from nakalab_ultralytics_interfaces.msg import ObjectSeg2DArray, ObjectSeg3DArray
from nakalab_ultralytics_interfaces.srv import Detect
import rclpy


class ObjectDetector:
    """
    同じノードの executor と逐次利用する物体検出クライアント.

    Parameters
    ----------
    node : rclpy.node.Node
        通信と時刻の取得に使うノード．検出ノードと use_sim_time を一致させる．
    service_timeout_sec : float, default 10.0
        サービス接続と応答の待機上限（壁時計秒）．

    """

    def __init__(self, node, service_timeout_sec=10.0):
        """開始・停止サービスと検出結果の購読を作成する."""
        if not math.isfinite(service_timeout_sec) or service_timeout_sec <= 0:
            raise ValueError('service_timeout_sec は正の有限値が必要です．')
        self.node = node
        self.timeout = service_timeout_sec
        self.client = node.create_client(Detect, 'nu_ros2/detect_object')
        self.result_2d = self.result_3d = None
        self.sub_2d = node.create_subscription(
            ObjectSeg2DArray, 'nu_ros2/object_seg_2d', self._on_2d, 10)
        self.sub_3d = node.create_subscription(
            ObjectSeg3DArray, 'nu_ros2/object_seg_3d', self._on_3d, 10)

    def _on_2d(self, message):
        self.result_2d = message

    def _on_3d(self, message):
        self.result_3d = message

    def execute(self, run, confidence=0.5):
        """モデルを開始・停止し，サービスの成功結果を返す."""
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('confidence は 0 以上 1 以下の有限値が必要です．')
        if not self.client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('物体検出サービスが起動していません．')
        future = self.client.call_async(
            Detect.Request(run=bool(run), confidence=float(confidence)))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
        if not future.done():
            future.cancel()
            raise RuntimeError('物体検出サービスがタイムアウトしました．')
        response = future.result()
        if response is None:
            raise RuntimeError('物体検出サービスから応答がありません．')
        if not response.success:
            self.node.get_logger().error(response.message)
        return response.success

    def detect(self, timeout_sec=10.0, confidence=0.5, class_names=None,
               require_3d=True, stop_after=True):
        """
        呼び出し後の画像に基づく検出結果を待つ.

        Parameters
        ----------
        timeout_sec : float, default 10.0
            検出開始後に結果を待つ壁時計秒数．
        confidence : float, default 0.5
            推論の信頼度しきい値．
        class_names : sequence of str or None, optional
            取得対象のクラス名．None は全クラス．
        require_3d : bool, default True
            有効な 3D 位置を持つ結果を待つ．False は 2D 結果を返す．
        stop_after : bool, default True
            成功・タイムアウト・例外時に検出モデルを停止する．

        Returns
        -------
        ObjectSeg2DArray or ObjectSeg3DArray or None
            取得時刻・座標系を含む検出配列．タイムアウト時は None．
            3D は valid=True の要素だけを返す．

        Raises
        ------
        RuntimeError
            開始・停止要求や通信が失敗した場合．
        ValueError
            引数が不正な場合．

        """
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('timeout_sec は正の有限値が必要です．')
        if isinstance(class_names, str):
            raise ValueError('class_names は文字列の配列を指定してください．')
        names = None if class_names is None else set(class_names)
        self.result_2d = self.result_3d = None
        earliest = self.node.get_clock().now().nanoseconds
        started = False
        try:
            started = self.execute(True, confidence)
            if not started:
                raise RuntimeError('物体検出を開始できませんでした．')
            deadline = time.monotonic() + timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(
                    self.node, timeout_sec=max(0.0, min(0.05, deadline - time.monotonic())))
                message = self.result_3d if require_3d else self.result_2d
                if message is None:
                    continue
                stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                if stamp < earliest:
                    continue
                selected = [obj for obj in message.objects
                            if (not require_3d or obj.valid)
                            and (names is None or obj.class_name in names)]
                if selected:
                    result = copy.deepcopy(message)
                    result.objects = copy.deepcopy(selected)
                    return result
            return None
        finally:
            if started and stop_after and not self.execute(False, confidence):
                raise RuntimeError('物体検出を停止できませんでした．')

    def close(self):
        """このクライアントの ROS 通信資源を解放する."""
        self.node.destroy_subscription(self.sub_2d)
        self.node.destroy_subscription(self.sub_3d)
        self.node.destroy_client(self.client)
