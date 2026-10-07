#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "nakalab_ultralytics_cpp/object_seg_fusion.hpp"

namespace fusion = nakalab_ultralytics_cpp::object_fusion;

class ObjectFusion : public ::testing::Test
{
protected:
  void SetUp() override
  {
    camera.width = 4;
    camera.height = 3;
    camera.k = {2, 0, 1, 0, 2, 1, 0, 0, 1};
    object.class_name = "cup";
    object.class_id = 41;
    object.confidence = 0.9F;
    object.roi.x_offset = 1;
    object.roi.width = 2;
    object.roi.height = 3;
    object.mask.encoding = "mono8";
    object.mask.width = 2;
    object.mask.height = 3;
    object.mask.step = 3;
    object.mask.data = {255, 255, 99, 255, 0, 99, 255, 255, 99};
    options.pixel_stride = 1;
    options.min_points = 1;
    fill_depth(false, false, std::vector<float>(12, 2.0F));
  }

  void fill_depth(bool floating, bool big, const std::vector<float> & values)
  {
    depth.width = 4;
    depth.height = 3;
    depth.encoding = floating ? "32FC1" : "16UC1";
    depth.is_bigendian = big;
    const size_t bytes = floating ? 4 : 2;
    depth.step = 4 * bytes + 2;
    depth.data.assign(depth.step * 3, 0xFF);
    for (size_t i = 0; i < values.size(); ++i) {
      uint32_t bits = 0;
      if (floating) {
        std::memcpy(&bits, &values[i], 4);
      } else {
        bits = static_cast<uint32_t>(values[i] * 1000);
      }
      for (size_t b = 0; b < bytes; ++b) {
        depth.data[(i / 4) * depth.step + (i % 4) * bytes + b] =
          (bits >> (8 * (big ? bytes - 1 - b : b))) & 0xFF;
      }
    }
  }

  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera;
  nakalab_ultralytics_interfaces::msg::ObjectSeg2D object;
  fusion::Options options;
};

TEST_F(ObjectFusion, DepthUnitsEndianPaddingAndMaskHoles)
{
  for (bool floating : {false, true}) {
    for (bool big : {false, true}) {
      fill_depth(floating, big, std::vector<float>(12, 2.0F));
      const auto points = fusion::project(object, depth, camera, options);
      ASSERT_EQ(points.size(), 5U);
      EXPECT_DOUBLE_EQ(points.front().x, 0.0);
      EXPECT_DOUBLE_EQ(points.front().y, -1.0);
      EXPECT_DOUBLE_EQ(points.front().z, 2.0);
      EXPECT_DOUBLE_EQ(points.back().x, 1.0);
      EXPECT_DOUBLE_EQ(points.back().y, 1.0);
    }
  }
}

TEST_F(ObjectFusion, RejectsInvalidDepthAndBackgroundOutlier)
{
  auto values = std::vector<float>(12, 2.0F);
  values[1] = 0;
  values[2] = std::numeric_limits<float>::quiet_NaN();
  values[10] = 5;
  fill_depth(true, false, values);
  const auto points = fusion::project(object, depth, camera, options);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points.front().z, 2.0);
  options.min_points = 3;
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
}

TEST_F(ObjectFusion, RejectsBoundsAndMalformedImages)
{
  object.roi.x_offset = std::numeric_limits<uint32_t>::max();
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
  object.roi.x_offset = 1;
  object.mask.data.pop_back();
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
  object.mask.data.push_back(0);
  depth.data.pop_back();
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
}

TEST_F(ObjectFusion, RejectsCalibrationAndOptions)
{
  camera.width = 5;
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
  camera.width = 4;
  camera.k[0] = 0;
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
  camera.k[0] = 2;
  options.pixel_stride = 0;
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
}

TEST_F(ObjectFusion, DistortionCorrectionChangesOffAxisPoint)
{
  const auto plain = fusion::project(object, depth, camera, options);
  camera.distortion_model = "plumb_bob";
  camera.d = {0.1, 0, 0, 0, 0};
  const auto corrected = fusion::project(object, depth, camera, options);
  EXPECT_LT(corrected.back().x, plain.back().x);
  EXPECT_DOUBLE_EQ(corrected.back().z, plain.back().z);
  camera.distortion_model = "unsupported";
  EXPECT_THROW(fusion::project(object, depth, camera, options), std::invalid_argument);
}

TEST_F(ObjectFusion, VisibleSurfaceSummaryAndInvalidStatus)
{
  auto result = fusion::invalid_object(object);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isnan(result.centroid.x));
  EXPECT_EQ(result.class_name, "cup");
  fusion::summarize(fusion::project(object, depth, camera, options), result);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.valid_points, 5U);
  EXPECT_DOUBLE_EQ(result.centroid.x, 0.4);
  EXPECT_DOUBLE_EQ(result.centroid.y, 0.0);
  EXPECT_DOUBLE_EQ(result.size.x, 1.0);
  EXPECT_DOUBLE_EQ(result.size.y, 2.0);
  EXPECT_DOUBLE_EQ(result.size.z, 0.0);
  EXPECT_THROW(fusion::summarize({}, result), std::invalid_argument);
}
