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

#include <memory>

#include "autoware/tensorrt_vad/utils.hpp"
#include "autoware/tensorrt_vad/vad_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace autoware::tensorrt_vad {

VadNode::VadNode(const rclcpp::NodeOptions &options)
    : Node("vad_node", options), tf_buffer_(this->get_clock()) {
  RCLCPP_INFO(this->get_logger(), "VAD Node initialized");
}

} // namespace autoware::tensorrt_vad

// Register the component with the ROS2 component system
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::VadNode)

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<autoware::tensorrt_vad::VadNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
