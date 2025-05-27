# C++ Naming Conventions for autoware_tensorrt_vad

## 必須ネーミング規則

このプロジェクトでは**clang-tidyの設定ファイル (`.clang-tidy`) で定義されたネーミング規則**を必ず遵守してください。

### clang-tidy設定の参照

プロジェクトルートの `.clang-tidy` ファイルに定義された `readability-identifier-naming` チェックの設定に従ってください：

```yaml
# 現在の設定 (.clang-tidy より)
CheckOptions:
  # Variables - lower_snake_case
  - key: readability-identifier-naming.VariableCase
    value: lower_case

  # Functions - lower_snake_case
  - key: readability-identifier-naming.FunctionCase
    value: lower_case

  # Methods - lower_snake_case (GoogleTest例外あり)
  - key: readability-identifier-naming.MethodCase
    value: lower_case
  - key: readability-identifier-naming.MethodIgnoredRegexp
    value: "^([a-z][a-z0-9_]*|SetUp|TearDown)$"

  # Classes - UpperCamelCase
  - key: readability-identifier-naming.ClassCase
    value: CamelCase

  # Structs - UpperCamelCase
  - key: readability-identifier-naming.StructCase
    value: CamelCase

  # Member variables - lower_snake_case_
  - key: readability-identifier-naming.MemberCase
    value: lower_case
  - key: readability-identifier-naming.MemberSuffix
    value: "_"

  # Parameters - lower_snake_case
  - key: readability-identifier-naming.ParameterCase
    value: lower_case

  # Enums - UpperCamelCase
  - key: readability-identifier-naming.EnumCase
    value: CamelCase

  # Enum constants - lower_snake_case (マクロ衝突回避)
  - key: readability-identifier-naming.EnumConstantCase
    value: lower_case

  # Constants - lower_snake_case (マクロ衝突回避)
  - key: readability-identifier-naming.ConstantCase
    value: lower_case

  # Namespaces - lower_case
  - key: readability-identifier-naming.NamespaceCase
    value: lower_case
```

## GoogleTest例外規則

GoogleTestフレームワークでは以下の例外が適用されます：

### 特別なメソッド

- `SetUp()`: テストフィクスチャの初期化（CamelCase必須）
- `TearDown()`: テストフィクスチャのクリーンアップ（CamelCase必須）

これらのメソッドはGoogleTestフレームワークが要求する命名規則のため、clang-tidyの設定で例外として許可されています。

### テスト名

- `TEST_F(TestFixture, TestName)`: テスト名は慣例的にCamelCaseを使用
- 例: `InitializationTest`, `InferenceTest`, `TrajectoryFormatTest`

### 例

```cpp
class VadModelTest : public ::testing::Test {
protected:
  // GoogleTest framework required methods - CamelCase naming is mandatory
  void SetUp() override {
    // テストの初期化
  }

  void TearDown() override {
    // テストのクリーンアップ
  }

  // 通常のヘルパーメソッドは lower_snake_case
  VadInputData create_dummy_input() {
    // ...
  }
};

// GoogleTest convention: Test names use CamelCase
TEST_F(VadModelTest, InitializationTest) {
  // テストの実装
}
```

## コード生成時の指示

**重要**: Claudeがコードを生成する際は、以下の手順に従ってください：

1. **`.clang-tidy`ファイルの設定を確認**してネーミング規則を把握する
2. **clang-tidyの`readability-identifier-naming`チェック**に準拠したコードを生成する
3. **GoogleTestの例外規則**を適用する：
   - `SetUp()`/`TearDown()`メソッド: CamelCase
   - テスト名: CamelCase
   - その他のヘルパーメソッド: lower_snake_case
4. 特に以下の点に注意：
   - メンバ変数: `lower_snake_case_` (末尾に`_`)
   - クラス/構造体: `UpperCamelCase`
   - 関数/メソッド/変数: `lower_snake_case`
   - enum定数: `lower_snake_case` (大文字にしない)

## 自動チェック

- **clang-tidy**: `.clang-tidy`設定でネーミング規則を自動チェック
- **pre-commit**: コミット時に自動的にclang-tidyが実行される
- **WarningsAsErrors**: ネーミング規則違反はエラーとして扱われる

## 設定確認コマンド

現在のclang-tidy設定を確認するには：

```bash
clang-tidy --config-file=.clang-tidy --dump-config | grep -A 50 "readability-identifier-naming"
```

## 参考資料

- マクロとの名前衝突回避: <https://ryo021021.hatenablog.com/entry/2013/11/12/130603>
- GoogleTest命名規則: <https://google.github.io/googletest/primer.html>
- clang-tidy設定ファイル: `.clang-tidy`
- pre-commit設定: `.pre-commit-config.yaml`
