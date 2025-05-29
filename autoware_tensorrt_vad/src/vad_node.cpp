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

#include "autoware/tensorrt_vad/vad_node.hpp"

#include "autoware/tensorrt_vad/utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <cmath>
#include <memory>

namespace autoware::tensorrt_vad
{

// ヘルパー関数：yaw角からクォータニオンを作成
geometry_msgs::msg::Quaternion calculate_quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q{};
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

VadNode::VadNode(const rclcpp::NodeOptions & options) : Node("vad_node", options)
{
  // Publishers の初期化
  trajectory_pub_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", rclcpp::QoS(1));

  // VADモデルの初期化
  vad_model_ptr_ = std::make_unique<VadModel>();

  // モデルファイルのパスを取得（パラメータから）
  std::string model_path = this->declare_parameter<std::string>("model_path", "/tmp/vad_model.onnx");

  // VADモデルの初期化
  if (vad_model_ptr_->initialize(model_path)) {
    RCLCPP_INFO(this->get_logger(), "VAD Model initialized successfully with: %s", model_path.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize VAD Model");
    return;
  }

  // TODO(Shin-kyoto): センサーデータのsubscriberを追加
  // - カメラ画像のsubscriber
  // - LiDARデータのsubscriber
  // - CAN-BUSデータのsubscriber

  RCLCPP_INFO(this->get_logger(), "VAD Node initialized");
}

void VadNode::execute_inference()
{
  // ダミーの入力データを作成（実際の実装では実際のセンサーデータを使用）
  VadInputData input_data;
  input_data.command_ = 1;  // デフォルトコマンド

  // VADモデルで推論を実行
  auto result = vad_model_->infer(input_data);

  if (result.has_value()) {
    RCLCPP_INFO(this->get_logger(), "VAD inference successful");

    // 軌道データのみを抽出してpublish
    publish_trajectory(result->predicted_trajectory_);
  } else {
    RCLCPP_WARN(this->get_logger(), "VAD inference failed");
  }
}

void VadNode::publish_trajectory(const std::vector<float> & predicted_trajectory)
{
  RCLCPP_INFO(this->get_logger(), "publish_trajectory called");

  auto trajectory_msg = std::make_unique<autoware_planning_msgs::msg::Trajectory>();

  double last_yaw = 0.0;  // 前の点の方向を記録

  for (size_t i = 0; i < predicted_trajectory.size(); i += 2) {
    autoware_planning_msgs::msg::TrajectoryPoint point;

    point.pose.position.x = predicted_trajectory[i];
    point.pose.position.y = predicted_trajectory[i + 1];
    point.pose.position.z = 0.0;

    if (i + 2 < predicted_trajectory.size()) {
      // 次の点との差分から方向を計算
      float dx = predicted_trajectory[i + 2] - predicted_trajectory[i];
      float dy = predicted_trajectory[i + 3] - predicted_trajectory[i + 1];
      last_yaw = std::atan2(dy, dx);
      point.pose.orientation = calculate_quaternion_from_yaw(last_yaw);
    } else {
      // 最後の点では前の方向を継続
      point.pose.orientation = calculate_quaternion_from_yaw(last_yaw);
    }

    point.longitudinal_velocity_mps = 0.0;
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;

    trajectory_msg->points.push_back(point);
  }

  trajectory_msg->header.stamp = this->now();
  // TODO(Shin-kyoto): 本来はmap座標系を使用するかを決定する
  trajectory_msg->header.frame_id = "base_link";

  trajectory_pub_->publish(std::move(trajectory_msg));

  RCLCPP_INFO(this->get_logger(), "Publish trajectory");
}

}  // namespace autoware::tensorrt_vad

// Register the component with the ROS2 component system
// NOLINTNEXTLINE(readability-identifier-naming,cppcoreguidelines-avoid-non-const-global-variables)
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::VadNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware::tensorrt_vad::VadNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
