# Coding Standards for autoware_tensorrt_vad

## 概要

このドキュメントは`autoware_tensorrt_vad`プロジェクトにおけるコーディング標準を定義します．GitHub CopilotやCursorを使用してコードを生成する際には，このドキュメントを使用してください．

## 1. C++標準 (C++ Standard)

### 1.1 使用するC++標準

このプロジェクトは**C++17**を使用します．これはAutoware Universeの標準に合わせています．

### 1.2 C++17機能の活用

以下のC++17機能を活用してください：

- **`std::optional`**: 失敗する可能性がある処理の返り値
- **`[[nodiscard]]`属性**: 重要な戻り値の無視を防ぐ
- [**構造化束縛**](https://cpprefjp.github.io/lang/cpp17/structured_bindings.html): タプルやペアの分解
- **`std::string_view`**: 文字列の効率的な参照

```cpp
// C++17機能の使用例
class VadModel {
public:
  // [[nodiscard]]で戻り値の無視を防ぐ
  [[nodiscard]] std::optional<VadOutputData> infer(const VadInputData& input);

  // std::string_viewで効率的な文字列処理
  [[nodiscard]] bool load_config(std::string_view config_path);
};

// 構造化束縛とstd::optionalの組み合わせ使用
auto result = model_->infer(input);
if (result.has_value()) {
  // std::optionalから値を取得
  const auto& [trajectory, confidence] = result.value();
  if (confidence > 0.8) {
    publish_trajectory(trajectory);
  }
} else {
  RCLCPP_ERROR_THROTTLE(logger_, *this->get_clock(), 1000, "Inference failed");
}
```

## 2. 命名規則 (Naming Conventions)

### 2.1 基本原則

- 可能な限り，ROS 2 humbleの従う命名規則を遵守します．

### 2.2 命名規則一覧

| 要素           | 規則                | 例                                         | 備考                     |
| -------------- | ------------------- | ------------------------------------------ | ------------------------ |
| **変数**       | `lower_snake_case`  | `model_path`, `input_data`                 | ローカル変数，パラメータ |
| **関数**       | `lower_snake_case`  | `initialize()`, `execute_inference()`      |                          |
| **メソッド**   | `lower_snake_case`  | `publish_trajectory()`, `is_initialized()` | クラスのメンバ関数       |
| **メンバ変数** | `lower_snake_case_` | `vad_model_`, `trajectory_pub_`            | 末尾に`_`必須            |
| **クラス**     | `UpperCamelCase`    | `VadModel`, `VadNode`                      |                          |
| **構造体**     | `UpperCamelCase`    | `VadInputData`, `VadOutputData`            |                          |
| **列挙型**     | `UpperCamelCase`    | `CommandType`, `InferenceMode`             |                          |
| **列挙値**     | `lower_snake_case`  | `turn_left`, `go_straight`                 | マクロ衝突回避           |
| **定数**       | `lower_snake_case`  | `max_trajectory_points`                    | マクロ衝突回避           |
| **名前空間**   | `lower_case`        | `autoware::tensorrt_vad`                   |                          |

### 2.3 GoogleTest例外規則

GoogleTestフレームワークでは以下の例外が適用されます：

#### 特別なメソッド

- `SetUp()`: テストフィクスチャの初期化（**CamelCase必須**）
- `TearDown()`: テストフィクスチャのクリーンアップ（**CamelCase必須**）

これらはGoogleTestフレームワークが要求する命名規則のため，clang-tidyの設定で例外として許可されています．

#### テスト名

- `TEST_F(TestFixture, TestName)`: テスト名は慣例的に**CamelCase**を使用
- 例: `InitializationTest`, `InferenceTest`, `TrajectoryFormatTest`

#### 例

```cpp
class VadModelTest : public ::testing::Test {
protected:
  // GoogleTest framework required methods - CamelCase naming is mandatory
  void SetUp() override {
    model_ = std::make_unique<VadModel>();
  }

  void TearDown() override {
    model_.reset();
  }

  // 通常のヘルパーメソッドは lower_snake_case
  VadInputData create_dummy_input() {
    // ...
  }

private:
  std::unique_ptr<VadModel> model_;  // メンバ変数は末尾に_
};

// GoogleTest convention: Test names use CamelCase
TEST_F(VadModelTest, InitializationTest) {
  EXPECT_TRUE(model_->initialize("/path/to/model"));
}
```

### 2.4 ヘッダーガード

予約識別子（連続するアンダースコア）を避けるため，以下の形式を使用します：

```cpp
// ❌ 予約識別子（使用禁止）
#ifndef AUTOWARE__TENSORRT_VAD__VAD_NODE_HPP_

// ✅ 推奨形式
#ifndef AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_
#define AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_
// ...
#endif // AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_
```

## 3. コマンドクエリの分離 (Command Query Separation)

### 3.1 基本原則

**コマンドクエリの分離**原則に従い，関数は以下のいずれかに分類されます：

- **コマンド**: 状態を変更するが値を返さない（`void`）
- **クエリ**: 状態を変更せず値を返す

### 3.2 処理を行う関数の返り値

処理を行う関数（推論，計算，変換など）は**必ず結果を返す**必要があります：

#### ✅ 推奨パターン

```cpp
// 推論結果を返す（失敗時はstd::nullopt）
std::optional<VadOutputData> infer(const VadInputData& input);

// 初期化結果を返す
bool initialize(const std::string& model_path);

// 変換結果を返す
std::optional<geometry_msgs::msg::Path> convert_to_path(const VadOutputData& output);

// 検証結果を返す
bool validate_input(const VadInputData& input);
```

#### ❌ 禁止パターン

```cpp
// ❌ 処理を行うのに返り値がvoid
void infer(const VadInputData& input);  // 結果がわからない

// ❌ 出力パラメータを使用
void infer(const VadInputData& input, VadOutputData& output);  // 古いCスタイル
```

### 3.3 std::optionalの使用

失敗する可能性がある処理には`std::optional`を使用します：

```cpp
class VadModel {
public:
  // 初期化（成功/失敗が明確）
  bool initialize(const std::string& model_path);

  // 推論（失敗時はstd::nullopt）
  std::optional<VadOutputData> infer(const VadInputData& input);

  // 状態確認（副作用なし）
  [[nodiscard]] bool is_initialized() const;

private:
  // 内部状態変更（返り値なし）
  void cleanup();
  void reset_internal_state();
};

// 使用例
auto result = model_->infer(input);
if (result.has_value()) {
  publish_trajectory(result.value());
} else {
  RCLCPP_ERROR(logger_, "Inference failed");
}
```

### 3.4 例外的なケース

以下の場合のみ`void`返り値が許可されます：

- **純粋なコマンド**: 状態変更のみで結果が不要
- **Publisher**: ROS2のpublish操作
- **ログ出力**: 副作用のみの操作

```cpp
// ✅ 許可される void メソッド
void publish_trajectory(const VadOutputData& output);  // Publisher
void log_performance_metrics();                        // ログ出力
void reset();                                          // 状態リセット
```

## 4. ロギングシステム (Logging System)

### 4.1 基本原則

- **ROS2ロギングシステム**を使用する
- **`std::cout`/`std::cerr`は禁止**（ログレベル制御不可のため）
- **スロットリング機能**を活用してログ量を制御する

### 4.2 ログレベルの使い分け

[ROS 2 humbleのSeverity levelを参照](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html#severity-level)

### 4.3 推奨ロギングパターン

#### 基本的なロギング

```cpp
class VadNode : public rclcpp::Node {
private:
  void some_method() {
    // 一般的な情報
    RCLCPP_INFO(this->get_logger(), "VAD inference started");

    // 警告
    RCLCPP_WARN(this->get_logger(), "Input data quality is low: score=%.2f", quality_score);

    // エラー
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize model: %s", error_msg.c_str());

    // デバッグ情報
    RCLCPP_DEBUG(this->get_logger(), "Processing %zu camera images", input.camera_images_.size());
  }
};
```

#### スロットリング（推奨）

高頻度で実行される処理では**必ずスロットリングを使用**します：

```cpp
class VadNode : public rclcpp::Node {
private:
  void execute_inference() {
    // 5秒間隔でログ出力（高頻度実行される推論処理）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Inference running at %.1f Hz", current_frequency);

    // 警告も同様にスロットリング
    if (inference_time > threshold) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                          "Inference time exceeded threshold: %.2f ms", inference_time);
    }

    // エラーは即座に出力（スロットリングなし）
    if (!result.has_value()) {
      RCLCPP_ERROR(this->get_logger(), "Inference failed for frame %d", frame_id);
    }
  }
};
```

#### 条件付きロギング

```cpp
// 条件付きログ出力
RCLCPP_INFO_EXPRESSION(this->get_logger(), enable_verbose_logging,
                      "Detailed processing info: %s", details.c_str());

// 一度だけ出力
RCLCPP_WARN_ONCE(this->get_logger(), "This warning will only appear once");
```

### 4.4 禁止事項

```cpp
// ❌ 禁止: std::cout/std::cerr
std::cout << "Processing data..." << std::endl;  // ログレベル制御不可
std::cerr << "Error occurred" << std::endl;      // ログレベル制御不可

// ❌ 禁止: printf系
printf("Debug info: %d\n", value);               // ログレベル制御不可

// ❌ 禁止: スロットリングなしの高頻度ログ
void high_frequency_callback() {
  RCLCPP_INFO(this->get_logger(), "Called");     // 毎回出力される
}
```

## 5. コードフォーマット

### 5.1 自動フォーマット

- **clang-format**: C++コードの自動フォーマット
- **prettier**: Markdown，YAML，JSONファイルのフォーマット

### 5.2 インデント

- **C++**: 2スペース
- **CMake**: 2スペース
- **YAML**: 2スペース

## 6. コメント規則

### 6.1 ファイルヘッダー

すべてのソースファイルには以下のライセンスヘッダーを含める：

```cpp
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
```

### 6.2 TODO コメント

```cpp
// TODO(username): 具体的な作業内容
// TODO(Shin-kyoto): TensorRTエンジンの初期化を実装
```

### 6.3 ドキュメンテーション

パブリックAPIには適切なコメントを記述：

```cpp
/**
 * @brief VADモデルを初期化する
 * @param model_path ONNXモデルファイルのパス
 * @return 初期化成功時はtrue，失敗時はfalse
 */
bool initialize(const std::string& model_path);
```

## 7. 自動チェック・品質保証

### 7.1 静的解析ツール

- **clang-tidy**: 命名規則，モダンC++，パフォーマンス，バグ検出
- **clang-format**: コードフォーマット
- **cpplint**: 無効化（clang-tidyに統一）

### 7.2 pre-commit フック

コミット時に以下が自動実行されます：

```yaml
# .pre-commit-config.yaml より抜粋
- repo: https://github.com/pocc/pre-commit-hooks
  hooks:
    - id: clang-tidy
      args: [--config-file=.clang-tidy]
    - id: clang-format
```

### 7.3 エラーとしての扱い

以下の違反はコンパイルエラーとして扱われます：

```yaml
# .clang-tidy より
WarningsAsErrors: "readability-identifier-naming"
```

## 8. 設定ファイル

### 8.1 主要設定ファイル

| ファイル                  | 用途             | 説明                         |
| ------------------------- | ---------------- | ---------------------------- |
| `.clang-tidy`             | 静的解析設定     | 命名規則，コード品質チェック |
| `.clang-format`           | フォーマット設定 | インデント，改行，スペース   |
| `.pre-commit-config.yaml` | pre-commitフック | 自動チェック設定             |

### 8.2 設定確認コマンド

```bash
# clang-tidy設定の確認
clang-tidy --config-file=.clang-tidy --dump-config | grep -A 50 "readability-identifier-naming"

# clang-format設定の確認
clang-format --dump-config
```

## 9. 開発ワークフロー

### 9.1 コード作成時

1. 命名規則に従ってコードを記述
   - このファイルをCursorやGitHub Copilotに読み込ませることで，coding standardsを遵守したコードを生成できます．
2. ローカルでbuildし，実行できることを確認
3. 適切なテストを追加

### 9.2 コミット前

1. pre-commitを実行
2. 違反があればコミットが拒否される
3. 修正後に再度コミット

### 9.3 CI/CD

1. GitHub Actionsで全ファイルをチェック
2. 違反があればPRがブロックされる

## 10. 例外とガイドライン

### 10.1 サードパーティライブラリ

- 外部ライブラリのAPIは元の命名規則に従う
- ラッパークラスでは本プロジェクトの規則に従う

### 10.2 レガシーコード

- 新規コードは必ず新しい規則に従う
- 既存コードの修正時は可能な限り新しい規則に合わせる

## 11. 参考資料

- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [GoogleTest Primer](https://google.github.io/googletest/primer.html)
- [clang-tidy documentation](https://clang.llvm.org/extra/clang-tidy/)
- [ROS2 Logging](https://docs.ros.org/en/humble/Concepts/About-Logging.html)
- [Command Query Separation](https://martinfowler.com/bliki/CommandQuerySeparation.html)
- [マクロとの名前衝突回避](https://ryo021021.hatenablog.com/entry/2013/11/12/130603)

## 12. 設定ファイルの場所

- **プロジェクトルート**: `.clang-tidy`, `.clang-format`, `.pre-commit-config.yaml`
- **開発者向けガイド**: `.cursor/rules/cpp_naming_conventions.md`
- **設計ドキュメント**: `docs/design/coding_standards.md` (このファイル)

---

**注意**: このドキュメントは`.clang-tidy`設定ファイルと同期して保持してください．設定変更時は両方のファイルを更新する必要があります．
