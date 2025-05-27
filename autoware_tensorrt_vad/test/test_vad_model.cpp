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

#include "autoware/tensorrt_vad/vad_trt.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>

namespace autoware::tensorrt_vad
{

class VadModelTest : public ::testing::Test
{
protected:
  // GoogleTest framework required methods - CamelCase naming is mandatory
  void SetUp() override { model_ = std::make_unique<VadModel>(); }

  void TearDown() override { model_.reset(); }

  // ダミーの入力データを生成するヘルパー関数
  VadInputData create_dummy_input()
  {
    VadInputData input;

    // ダミーのカメラ画像データ（6台のカメラを想定）
    input.camera_images_.resize(6);
    for (auto & camera_image : input.camera_images_) {
      camera_image.resize(224 * 224 * 3);  // 仮のサイズ
      std::fill(camera_image.begin(), camera_image.end(), 0.5f);
    }

    // ダミーのBEV特徴量
    input.prev_bev_.resize(100 * 100 * 64);  // 仮のサイズ
    std::fill(input.prev_bev_.begin(), input.prev_bev_.end(), 0.0f);

    // ダミーのシフトデータ
    input.shift_data_ = {0.0f, 0.0f};

    // ダミーのlidar2imgデータ（6台のカメラ × 4x4行列）
    input.lidar2img_data_.resize(6 * 16);
    std::fill(input.lidar2img_data_.begin(), input.lidar2img_data_.end(), 0.0f);

    // ダミーのCAN-BUSデータ
    input.can_bus_data_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // タイムスタンプ
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    input.timestamp_ = std::chrono::duration<double>(duration).count();

    // コマンドインデックス（0-2の範囲でランダム）
    input.command_ = static_cast<int32_t>(input.timestamp_) % 3;

    return input;
  }

  std::unique_ptr<VadModel> model_;
};

// GoogleTest convention: Test names use CamelCase
// VadModelの基本的な初期化テスト
TEST_F(VadModelTest, InitializationTest)
{
  // 初期状態では初期化されていない
  EXPECT_FALSE(model_->is_initialized());

  // ダミーのモデルパスで初期化
  std::string dummy_model_path = "/tmp/dummy_vad_model.onnx";
  bool result = model_->initialize(dummy_model_path);

  // ダミー実装では常に成功する
  EXPECT_TRUE(result);
  EXPECT_TRUE(model_->is_initialized());
}

// VadModelの推論テスト
TEST_F(VadModelTest, InferenceTest)
{
  // モデルを初期化
  ASSERT_TRUE(model_->initialize("/tmp/dummy_model.onnx"));

  // ダミー入力データを作成
  VadInputData input = create_dummy_input();

  // 推論を実行
  auto result = model_->infer(input);

  // 結果が有効であることを確認
  ASSERT_TRUE(result.has_value());

  const auto & output = result.value();

  // 出力データの検証
  EXPECT_EQ(output.predicted_trajectory_.size(), 12);  // 6点 × 2座標
  EXPECT_EQ(output.selected_command_index_, input.command_);
  EXPECT_GT(output.inference_time_ms_, 0.0);  // 推論時間が正の値
  EXPECT_FALSE(output.detected_objects_.empty());
  EXPECT_FALSE(output.confidence_scores_.empty());
}

// 初期化されていないモデルでの推論テスト
TEST_F(VadModelTest, InferenceWithoutInitializationTest)
{
  // 初期化せずに推論を実行
  VadInputData input = create_dummy_input();
  auto result = model_->infer(input);

  // 結果が無効であることを確認
  EXPECT_FALSE(result.has_value());
}

// 軌道データの形式テスト
TEST_F(VadModelTest, TrajectoryFormatTest)
{
  // モデルを初期化
  ASSERT_TRUE(model_->initialize("/tmp/dummy_model.onnx"));

  // 推論を実行
  VadInputData input = create_dummy_input();
  auto result = model_->infer(input);

  ASSERT_TRUE(result.has_value());

  const auto & trajectory = result.value().predicted_trajectory_;

  // 軌道が6つの2D座標点（12要素）であることを確認
  ASSERT_EQ(trajectory.size(), 12);

  // 各座標点が有効な値であることを確認
  for (size_t i = 0; i < trajectory.size(); i += 2) {
    float x = trajectory[i];
    float y = trajectory[i + 1];

    // 座標値が有限であることを確認
    EXPECT_TRUE(std::isfinite(x));
    EXPECT_TRUE(std::isfinite(y));
  }
}

// 複数回の推論テスト
TEST_F(VadModelTest, MultipleInferenceTest)
{
  // モデルを初期化
  ASSERT_TRUE(model_->initialize("/tmp/dummy_model.onnx"));

  // 複数回推論を実行
  for (int i = 0; i < 5; ++i) {
    VadInputData input = create_dummy_input();
    input.command_ = i % 3;  // コマンドを変更

    auto result = model_->infer(input);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value().selected_command_index_, input.command_);
  }
}

}  // namespace autoware::tensorrt_vad

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
