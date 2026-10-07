cmake_minimum_required(VERSION 3.8)
project(nakalab_ultralytics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nakalab_ultralytics_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d)

add_library(object_seg_fusion STATIC src/object_seg_fusion.cpp)
target_compile_features(object_seg_fusion PUBLIC cxx_std_17)
target_include_directories(object_seg_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(object_seg_fusion geometry_msgs nakalab_ultralytics_interfaces sensor_msgs)
target_link_libraries(object_seg_fusion ${OpenCV_LIBRARIES})

add_executable(object_seg_pose_3d src/object_seg_pose_3d.cpp)
target_compile_features(object_seg_pose_3d PUBLIC cxx_std_17)
target_link_libraries(object_seg_pose_3d object_seg_fusion)
ament_target_dependencies(object_seg_pose_3d
  geometry_msgs message_filters nakalab_ultralytics_interfaces rclcpp
  sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)

add_library(person_pose_fusion STATIC src/person_pose_fusion.cpp)
target_compile_features(person_pose_fusion PUBLIC cxx_std_17)
target_include_directories(person_pose_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(person_pose_fusion
  geometry_msgs
  nakalab_ultralytics_interfaces
  sensor_msgs
  tf2_geometry_msgs
)

add_executable(person_pose_3d src/person_pose_3d.cpp)
target_compile_features(person_pose_3d PUBLIC cxx_std_17)
target_include_directories(person_pose_3d PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(person_pose_3d person_pose_fusion)
ament_target_dependencies(person_pose_3d
  geometry_msgs
  message_filters
  nakalab_ultralytics_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  person_pose_3d
  object_seg_pose_3d
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_object_seg_fusion test/test_object_seg_fusion.cpp)
  if(TARGET test_object_seg_fusion)
    target_link_libraries(test_object_seg_fusion object_seg_fusion)
  endif()

  ament_add_gtest(test_person_pose_fusion test/test_person_pose_fusion.cpp)
  if(TARGET test_person_pose_fusion)
    target_compile_features(test_person_pose_fusion PUBLIC cxx_std_17)
    target_include_directories(test_person_pose_fusion PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(test_person_pose_fusion person_pose_fusion)
    ament_target_dependencies(test_person_pose_fusion
      geometry_msgs
      nakalab_ultralytics_interfaces
      sensor_msgs
    )
  endif()
endif()

ament_package()
