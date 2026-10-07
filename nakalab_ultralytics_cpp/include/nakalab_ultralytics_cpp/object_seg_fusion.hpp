#ifndef NAKALAB_ULTRALYTICS_CPP__OBJECT_SEG_FUSION_HPP_
#define NAKALAB_ULTRALYTICS_CPP__OBJECT_SEG_FUSION_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nakalab_ultralytics_interfaces/msg/object_seg2_d.hpp"
#include "nakalab_ultralytics_interfaces/msg/object_seg3_d.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace nakalab_ultralytics_cpp::object_fusion
{
struct Options
{
  double min_depth_m{0.1};
  double max_depth_m{6.0};
  double depth_outlier_floor_m{0.02};
  int pixel_stride{2};
  int min_points{12};
};

// 位置合わせ済みの深度と，元画像上のマスクから可視表面の点群を作る．
std::vector<geometry_msgs::msg::Point> project(
  const nakalab_ultralytics_interfaces::msg::ObjectSeg2D & object,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera,
  const Options & options);

// 出力座標系へ変換済みの点群から代表位置と軸平行寸法を求める．
void summarize(
  const std::vector<geometry_msgs::msg::Point> & points,
  nakalab_ultralytics_interfaces::msg::ObjectSeg3D & output);

nakalab_ultralytics_interfaces::msg::ObjectSeg3D invalid_object(
  const nakalab_ultralytics_interfaces::msg::ObjectSeg2D & object);
}  // namespace nakalab_ultralytics_cpp::object_fusion

#endif  // NAKALAB_ULTRALYTICS_CPP__OBJECT_SEG_FUSION_HPP_
