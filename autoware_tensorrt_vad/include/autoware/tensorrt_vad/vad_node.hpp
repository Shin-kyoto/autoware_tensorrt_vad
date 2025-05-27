// Copyright 2025 Shin-kyoto.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__TENSORRT_VAD__VAD_NODE_HPP_
#define AUTOWARE__TENSORRT_VAD__VAD_NODE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// Forward declarations for future use
// #include <cuda_runtime.h>
// #include <NvInfer.h>
// #include <stb/stb_image.h>
// #include <sensor_msgs/msg/image.hpp>
// #include <autoware_perception_msgs/msg/detected_objects.hpp>

namespace autoware::tensorrt_vad {
class VadNode : public rclcpp::Node {
public:
  explicit VadNode(const rclcpp::NodeOptions &options);

private:
  tf2_ros::Buffer tf_buffer_; // TF buffer for coordinate transformations
};
} // namespace autoware::tensorrt_vad

#endif // AUTOWARE__TENSORRT_VAD__VAD_NODE_HPP_
