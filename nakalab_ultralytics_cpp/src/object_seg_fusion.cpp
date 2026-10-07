#include "nakalab_ultralytics_cpp/object_seg_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"

namespace nakalab_ultralytics_cpp::object_fusion
{
namespace
{
double median(std::vector<double> values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  return values.size() % 2 ? upper : (upper + *std::max_element(values.begin(), middle)) / 2.0;
}

double depth_at(const sensor_msgs::msg::Image & image, uint32_t u, uint32_t v)
{
  const bool integer = image.encoding == "16UC1";
  const size_t size = integer ? 2 : 4;
  const auto offset = static_cast<size_t>(v) * image.step + u * size;
  uint32_t bits = 0;
  for (size_t i = 0; i < size; ++i) {
    const size_t shift = image.is_bigendian ? size - 1 - i : i;
    bits |= static_cast<uint32_t>(image.data[offset + i]) << (8 * shift);
  }
  if (integer) {
    return bits * 0.001;
  }
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

std::vector<geometry_msgs::msg::Point> project(
  const nakalab_ultralytics_interfaces::msg::ObjectSeg2D & object,
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::CameraInfo & camera,
  const Options & options)
{
  if (options.pixel_stride < 1 || options.min_points < 1 ||
    !std::isfinite(options.min_depth_m) || !std::isfinite(options.max_depth_m) ||
    options.min_depth_m <= 0 || options.max_depth_m <= options.min_depth_m ||
    !std::isfinite(options.depth_outlier_floor_m) || options.depth_outlier_floor_m <= 0)
  {
    throw std::invalid_argument("深度・標本化の設定が不正です．");
  }
  if (depth.encoding != "16UC1" && depth.encoding != "32FC1") {
    throw std::invalid_argument("深度は 16UC1 または 32FC1 が必要です．");
  }
  const size_t bytes = depth.encoding == "16UC1" ? 2 : 4;
  if (depth.width == 0 || depth.height == 0 ||
    depth.step < static_cast<uint64_t>(depth.width) * bytes ||
    depth.data.size() < static_cast<uint64_t>(depth.step) * depth.height ||
    camera.width != depth.width || camera.height != depth.height ||
    !std::all_of(camera.k.begin(), camera.k.end(), [](double v) {return std::isfinite(v);}) ||
    camera.k[0] <= 0 || camera.k[4] <= 0 || camera.binning_x > 1 || camera.binning_y > 1 ||
    camera.roi.x_offset != 0 || camera.roi.y_offset != 0 ||
    (camera.roi.width != 0 && camera.roi.width != camera.width) ||
    (camera.roi.height != 0 && camera.roi.height != camera.height))
  {
    throw std::invalid_argument("深度画像と CameraInfo の寸法・内部パラメータが不正です．");
  }
  const auto & roi = object.roi;
  const auto & mask = object.mask;
  if (roi.width == 0 || roi.height == 0 ||
    static_cast<uint64_t>(roi.x_offset) + roi.width > depth.width ||
    static_cast<uint64_t>(roi.y_offset) + roi.height > depth.height ||
    mask.encoding != "mono8" || mask.width != roi.width || mask.height != roi.height ||
    mask.step < mask.width || mask.data.size() < static_cast<uint64_t>(mask.step) * mask.height)
  {
    throw std::invalid_argument("元画像上の領域と切り出しマスクが一致しません．");
  }

  std::vector<cv::Point2d> pixels;
  std::vector<double> depths;
  for (uint32_t y = 0; y < roi.height; y += options.pixel_stride) {
    for (uint32_t x = 0; x < roi.width; x += options.pixel_stride) {
      if (mask.data[static_cast<size_t>(y) * mask.step + x] == 0) {
        continue;
      }
      const auto u = roi.x_offset + x;
      const auto v = roi.y_offset + y;
      const double z = depth_at(depth, u, v);
      if (std::isfinite(z) && z >= options.min_depth_m && z <= options.max_depth_m) {
        pixels.emplace_back(u, v);
        depths.push_back(z);
      }
    }
  }
  if (depths.size() < static_cast<size_t>(options.min_points)) {
    throw std::invalid_argument("マスク内の有効深度点が不足しています．");
  }
  cv::Mat intrinsics(3, 3, CV_64F);
  std::copy(camera.k.begin(), camera.k.end(), intrinsics.ptr<double>());
  if (!std::all_of(camera.d.begin(), camera.d.end(), [](double v) {return std::isfinite(v);})) {
    throw std::invalid_argument("歪み係数が不正です．");
  }
  std::vector<cv::Point2d> rays;
  if (camera.distortion_model == "equidistant" && camera.d.size() == 4) {
    cv::fisheye::undistortPoints(pixels, rays, intrinsics, camera.d);
  } else if ((camera.distortion_model == "plumb_bob" ||
    camera.distortion_model == "rational_polynomial" || camera.distortion_model.empty()) &&
    (camera.d.empty() || camera.d.size() == 4 || camera.d.size() == 5 ||
    camera.d.size() == 8 || camera.d.size() == 12 || camera.d.size() == 14))
  {
    cv::undistortPoints(pixels, rays, intrinsics, camera.d);
  } else {
    throw std::invalid_argument("未対応の歪みモデルです．");
  }
  const double center = median(depths);
  std::vector<double> deviations;
  for (double z : depths) {
    deviations.push_back(std::abs(z - center));
  }
  const double limit = std::max(options.depth_outlier_floor_m, 3.0 * 1.4826 * median(deviations));
  std::vector<geometry_msgs::msg::Point> points;
  for (size_t i = 0; i < depths.size(); ++i) {
    if (std::abs(depths[i] - center) > limit ||
      !std::isfinite(rays[i].x) || !std::isfinite(rays[i].y))
    {
      continue;
    }
    geometry_msgs::msg::Point point;
    point.x = rays[i].x * depths[i];
    point.y = rays[i].y * depths[i];
    point.z = depths[i];
    points.push_back(point);
  }
  if (points.size() < static_cast<size_t>(options.min_points)) {
    throw std::invalid_argument("外れ値除去後の有効深度点が不足しています．");
  }
  return points;
}

nakalab_ultralytics_interfaces::msg::ObjectSeg3D invalid_object(
  const nakalab_ultralytics_interfaces::msg::ObjectSeg2D & object)
{
  nakalab_ultralytics_interfaces::msg::ObjectSeg3D out;
  out.detection_id = object.detection_id;
  out.class_id = object.class_id;
  out.class_name = object.class_name;
  out.confidence = object.confidence;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  out.centroid.x = out.centroid.y = out.centroid.z = nan;
  out.box_center = out.centroid;
  out.size.x = out.size.y = out.size.z = nan;
  return out;
}

void summarize(
  const std::vector<geometry_msgs::msg::Point> & points,
  nakalab_ultralytics_interfaces::msg::ObjectSeg3D & output)
{
  if (points.empty()) {
    throw std::invalid_argument("点群が空です．");
  }
  auto low = points.front();
  auto high = low;
  geometry_msgs::msg::Point sum;
  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::invalid_argument("変換後の点群が有限値ではありません．");
    }
    sum.x += point.x;
    sum.y += point.y;
    sum.z += point.z;
    low.x = std::min(low.x, point.x);
    low.y = std::min(low.y, point.y);
    low.z = std::min(low.z, point.z);
    high.x = std::max(high.x, point.x);
    high.y = std::max(high.y, point.y);
    high.z = std::max(high.z, point.z);
  }
  output.centroid.x = sum.x / points.size();
  output.centroid.y = sum.y / points.size();
  output.centroid.z = sum.z / points.size();
  output.box_center.x = (low.x + high.x) / 2;
  output.box_center.y = (low.y + high.y) / 2;
  output.box_center.z = (low.z + high.z) / 2;
  output.size.x = high.x - low.x;
  output.size.y = high.y - low.y;
  output.size.z = high.z - low.z;
  output.valid_points = points.size();
  output.valid = true;
  output.reason.clear();
}
}  // namespace nakalab_ultralytics_cpp::object_fusion
