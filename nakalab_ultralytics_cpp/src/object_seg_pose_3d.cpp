#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nakalab_ultralytics_cpp/object_seg_fusion.hpp"
#include "nakalab_ultralytics_interfaces/msg/object_seg2_d_array.hpp"
#include "nakalab_ultralytics_interfaces/msg/object_seg3_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace fusion = nakalab_ultralytics_cpp::object_fusion;
using Objects2D = nakalab_ultralytics_interfaces::msg::ObjectSeg2DArray;
using Objects3D = nakalab_ultralytics_interfaces::msg::ObjectSeg3DArray;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using Marker = visualization_msgs::msg::Marker;
using Markers = visualization_msgs::msg::MarkerArray;

class ObjectSegPose3D : public rclcpp::Node
{
public:
  ObjectSegPose3D()
  : Node("object_seg_pose_3d")
  {
    ref_frame_ = declare_parameter<std::string>("ref_frame", "base_link");
    tolerance_ = declare_parameter<double>("sync_tolerance_sec", 0.1);
    stale_timeout_ = declare_parameter<double>("stale_timeout_sec", 2.0);
    options_.min_depth_m = declare_parameter<double>("min_depth_m", 0.1);
    options_.max_depth_m = declare_parameter<double>("max_depth_m", 6.0);
    options_.depth_outlier_floor_m = declare_parameter<double>("depth_outlier_floor_m", 0.02);
    options_.pixel_stride = declare_parameter<int>("pixel_stride", 2);
    options_.min_points = declare_parameter<int>("min_points", 12);
    if (!std::isfinite(tolerance_) || tolerance_ <= 0 ||
      !std::isfinite(stale_timeout_) || stale_timeout_ <= 0 ||
      !std::isfinite(options_.min_depth_m) || !std::isfinite(options_.max_depth_m) ||
      options_.min_depth_m <= 0 || options_.max_depth_m <= options_.min_depth_m ||
      options_.min_points < 1 || options_.pixel_stride < 1 ||
      !std::isfinite(options_.depth_outlier_floor_m) || options_.depth_outlier_floor_m <= 0)
    {
      throw std::invalid_argument("3D 変換パラメータが不正です．");
    }
    output_ = create_publisher<Objects3D>("nu_ros2/object_seg_3d", 10);
    markers_ = create_publisher<Markers>("nu_ros2/object_markers", 10);
    poses_ = create_publisher<geometry_msgs::msg::PoseArray>("nu_ros2/object_poses", 10);
    camera_sub_ = create_subscription<CameraInfo>(
      "color_camera_info", rclcpp::SensorDataQoS(),
      [this](CameraInfo::ConstSharedPtr info) {camera_ = info;});
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    objects_sub_.subscribe(this, "nu_ros2/object_seg_2d", rclcpp::QoS(10).get_rmw_qos_profile());
    depth_sub_.subscribe(this, "depth_image", rmw_qos_profile_sensor_data);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(30), objects_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance_));
    sync_->registerCallback(
      std::bind(
        &ObjectSegPose3D::on_pair, this, std::placeholders::_1, std::placeholders::_2));
    // 深度が届かない場合も，検出ゼロ・停止を直ちに伝える．
    objects_sub_.registerCallback(
      [this](Objects2D::ConstSharedPtr msg) {
        if (msg->objects.empty()) {
          empty_barrier_ = rclcpp::Time(msg->header.stamp).nanoseconds();
          Objects3D empty;
          empty.header = msg->header;
          empty.source_header = msg->header;
          if (!ref_frame_.empty()) {empty.header.frame_id = ref_frame_;}
          publish(empty);
        }
      });
    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(250), [this]() {
        if (has_objects_ && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_output_).count() > stale_timeout_)
        {
          Objects3D empty;
          empty.header = last_header_;
          empty.source_header = last_source_header_;
          publish(empty);
        }
      });
  }

private:
  using Policy = message_filters::sync_policies::ApproximateTime<Objects2D, Image>;

  void on_pair(Objects2D::ConstSharedPtr objects, Image::ConstSharedPtr depth)
  {
    const auto stamp = rclcpp::Time(objects->header.stamp).nanoseconds();
    if (empty_barrier_ && stamp <= *empty_barrier_ && !objects->objects.empty()) {
      return;
    }
    Objects3D result;
    result.source_header = objects->header;
    result.header = objects->header;
    if (!ref_frame_.empty()) {result.header.frame_id = ref_frame_;}
    std::string error;
    std::optional<geometry_msgs::msg::TransformStamped> transform;
    try {
      if (objects->header.frame_id.empty() || !camera_ ||
        camera_->header.frame_id != objects->header.frame_id ||
        depth->header.frame_id != objects->header.frame_id ||
        objects->image_width != depth->width || objects->image_height != depth->height)
      {
        throw std::invalid_argument("RGB・整列深度・CameraInfo の座標系または寸法が不一致です．");
      }
      if (std::abs(
          (rclcpp::Time(objects->header.stamp) -
          rclcpp::Time(depth->header.stamp)).seconds()) > tolerance_)
      {
        throw std::invalid_argument("RGB と深度の時刻差が許容範囲外です．");
      }
      if (result.header.frame_id != objects->header.frame_id) {
        // 時刻 0 は TF の最新指定となるため，時刻付きの結果へ置き換えない．
        if (stamp == 0) {throw std::invalid_argument("TF 変換には非ゼロの画像時刻が必要です．");}
        transform = buffer_->lookupTransform(
          result.header.frame_id, objects->header.frame_id,
          rclcpp::Time(objects->header.stamp), rclcpp::Duration::from_seconds(0.1));
      }
    } catch (const std::exception & exc) {
      error = exc.what();
    }
    for (const auto & object : objects->objects) {
      auto output = fusion::invalid_object(object);
      try {
        if (!error.empty()) {throw std::invalid_argument(error);}
        if (object.mask.header != objects->header) {
          throw std::invalid_argument("マスクと元画像のヘッダーが一致しません．");
        }
        auto points = fusion::project(object, *depth, *camera_, options_);
        if (transform) {
          for (auto & point : points) {
            geometry_msgs::msg::PointStamped source, target;
            source.header = objects->header;
            source.point = point;
            tf2::doTransform(source, target, *transform);
            point = target.point;
          }
        }
        fusion::summarize(points, output);
      } catch (const std::exception & exc) {
        output.reason = exc.what();
      }
      result.objects.push_back(output);
    }
    publish(result);
  }

  void publish(const Objects3D & result)
  {
    output_->publish(result);
    Markers markers;
    Marker clear;
    clear.header = result.header;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    geometry_msgs::msg::PoseArray poses;
    poses.header = result.header;
    for (const auto & object : result.objects) {
      if (!object.valid) {continue;}
      geometry_msgs::msg::Pose pose;
      pose.position = object.centroid;
      pose.orientation.w = 1.0;
      poses.poses.push_back(pose);
      Marker marker;
      marker.header = result.header;
      marker.ns = "objects";
      marker.id = object.detection_id;
      marker.type = Marker::CUBE;
      marker.action = Marker::ADD;
      marker.pose.position = object.box_center;
      marker.pose.orientation.w = 1.0;
      marker.scale = object.size;
      // 平面状の可視表面も RViz 上で確認できるよう，描画だけ最小厚さを設ける．
      marker.scale.x = std::max(0.005, marker.scale.x);
      marker.scale.y = std::max(0.005, marker.scale.y);
      marker.scale.z = std::max(0.005, marker.scale.z);
      marker.color.g = 0.8F;
      marker.color.b = 0.3F;
      marker.color.a = 0.5F;
      marker.lifetime = rclcpp::Duration::from_seconds(stale_timeout_);
      markers.markers.push_back(marker);
    }
    markers_->publish(markers);
    poses_->publish(poses);
    last_header_ = result.header;
    last_source_header_ = result.source_header;
    last_output_ = std::chrono::steady_clock::now();
    has_objects_ = !result.objects.empty();
  }

  fusion::Options options_;
  std::string ref_frame_;
  double tolerance_, stale_timeout_;
  CameraInfo::ConstSharedPtr camera_;
  message_filters::Subscriber<Objects2D> objects_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_sub_;
  rclcpp::Publisher<Objects3D>::SharedPtr output_;
  rclcpp::Publisher<Markers>::SharedPtr markers_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std_msgs::msg::Header last_header_, last_source_header_;
  std::chrono::steady_clock::time_point last_output_;
  std::optional<int64_t> empty_barrier_;
  bool has_objects_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObjectSegPose3D>());
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(rclcpp::get_logger("object_seg_pose_3d"), "%s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
