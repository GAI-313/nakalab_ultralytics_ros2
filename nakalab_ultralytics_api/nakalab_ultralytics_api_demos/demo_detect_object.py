#!/usr/bin/env python3
"""物体を検出し，基準座標系の可視表面位置と寸法を表示する."""

from nakalab_ultralytics_api.object_detector import ObjectDetector
import rclpy
from rclpy.node import Node


def main(args=None):
    """物体検出を開始し，結果を表示して停止する."""
    rclpy.init(args=args)
    node = Node('demo_detect_object')
    detector = ObjectDetector(node)
    try:
        result = detector.detect()
        if result is None:
            node.get_logger().warning('有効な 3D 検出結果を取得できませんでした．')
            return 1
        for obj in result.objects:
            point, size = obj.centroid, obj.size
            node.get_logger().info(
                f'{obj.class_name}: confidence={obj.confidence:.3f}, '
                f'frame={result.header.frame_id}, '
                f'position=({point.x:.3f}, {point.y:.3f}, {point.z:.3f}) m, '
                f'size=({size.x:.3f}, {size.y:.3f}, {size.z:.3f}) m')
        return 0
    finally:
        detector.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
