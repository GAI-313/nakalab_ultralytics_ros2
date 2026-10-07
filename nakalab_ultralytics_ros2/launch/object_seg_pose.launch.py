#!/usr/bin/env python3
"""RGB-D による物体セグメンテーションと可視表面の位置推定を起動する."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # launch 作成規約のセクション間隔に限り E303 の対象外とする．
    ld = LaunchDescription()


    # 既定値  # noqa: E303
    default_model_path = '/tmp/yolo26l-seg.pt'


    # 起動設定  # noqa: E303
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    run_detect = LaunchConfiguration('run_detect')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_3d = LaunchConfiguration('enable_3d')
    ref_frame = LaunchConfiguration('ref_frame')
    color_image = LaunchConfiguration('color_image')
    depth_image = LaunchConfiguration('depth_image')
    color_camera_info = LaunchConfiguration('color_camera_info')
    confidence = LaunchConfiguration('confidence')
    imgsz = LaunchConfiguration('imgsz')
    max_det = LaunchConfiguration('max_det')
    max_rate_hz = LaunchConfiguration('max_rate_hz')
    sync_tolerance_sec = LaunchConfiguration('sync_tolerance_sec')


    # 起動引数  # noqa: E303
    declare_model_path = DeclareLaunchArgument(
        'model_path', default_value=default_model_path, description='配置済み seg モデルのパス')
    declare_device = DeclareLaunchArgument('device', default_value='cuda', description='推論デバイス')
    declare_run_detect = DeclareLaunchArgument(
        'run_detect', default_value='false', choices=['true', 'false'], description='起動時に推論を開始')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=['true', 'false'], description='ROS の仮想時刻')
    declare_enable_3d = DeclareLaunchArgument(
        'enable_3d', default_value='true', choices=['true', 'false'], description='深度から 3D 位置を算出')
    declare_ref_frame = DeclareLaunchArgument(
        'ref_frame', default_value='base_link', description='3D 出力座標系．空文字ならカメラ座標系')
    declare_color_image = DeclareLaunchArgument(
        'color_image', default_value='/head_camera/d455/color/image_raw', description='RGB 画像')
    declare_depth_image = DeclareLaunchArgument(
        'depth_image', default_value='/head_camera/d455/aligned_depth_to_color/image_raw',
        description='RGB に位置合わせ済みの深度画像')
    declare_color_camera_info = DeclareLaunchArgument(
        'color_camera_info', default_value='/head_camera/d455/color/camera_info',
        description='RGB 画像の内部パラメータ')
    declare_confidence = DeclareLaunchArgument(
        'confidence', default_value='0.5', description='起動時の信頼度しきい値')
    declare_imgsz = DeclareLaunchArgument('imgsz', default_value='640', description='推論画像サイズ')
    declare_max_det = DeclareLaunchArgument('max_det', default_value='20', description='最大検出数')
    declare_max_rate_hz = DeclareLaunchArgument(
        'max_rate_hz', default_value='10.0', description='最大処理頻度 Hz')
    declare_sync_tolerance_sec = DeclareLaunchArgument(
        'sync_tolerance_sec', default_value='0.1', description='RGB と深度の時刻差上限 秒')
    ld.add_action(declare_model_path)
    ld.add_action(declare_device)
    ld.add_action(declare_run_detect)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_3d)
    ld.add_action(declare_ref_frame)
    ld.add_action(declare_color_image)
    ld.add_action(declare_depth_image)
    ld.add_action(declare_color_camera_info)
    ld.add_action(declare_confidence)
    ld.add_action(declare_imgsz)
    ld.add_action(declare_max_det)
    ld.add_action(declare_max_rate_hz)
    ld.add_action(declare_sync_tolerance_sec)


    # ノード  # noqa: E303
    remappings = [('color_image', color_image), ('depth_image', depth_image),
                  ('color_camera_info', color_camera_info)]
    segmentation = Node(
        package='nakalab_ultralytics_ros2', executable='object_seg_pose',
        output='screen', emulate_tty=True, remappings=remappings,
        parameters=[{
            'model_path': ParameterValue(model_path, value_type=str),
            'device': ParameterValue(device, value_type=str),
            'run_detect': ParameterValue(run_detect, value_type=bool),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'confidence': ParameterValue(confidence, value_type=float),
            'imgsz': ParameterValue(imgsz, value_type=int),
            'max_det': ParameterValue(max_det, value_type=int),
            'max_rate_hz': ParameterValue(max_rate_hz, value_type=float),
        }])
    projection = Node(
        package='nakalab_ultralytics_cpp', executable='object_seg_pose_3d',
        output='screen', emulate_tty=True, remappings=remappings,
        condition=IfCondition(enable_3d), parameters=[{
            'ref_frame': ParameterValue(ref_frame, value_type=str),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'sync_tolerance_sec': ParameterValue(sync_tolerance_sec, value_type=float),
        }])
    ld.add_action(segmentation)
    ld.add_action(projection)


    return ld  # noqa: E303
