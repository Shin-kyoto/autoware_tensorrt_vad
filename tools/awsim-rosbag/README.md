# AWSIM ROSbag CLI

AWSIM ROSbagの記録と変換を行うCLIツールです。

## セットアップ方法

### 環境のセットアップ

```sh
~/tools/awsim-rosbag
❯ uv venv
Using CPython 3.12.5
Creating virtual environment at: .venv
Activate with: source .venv/bin/activate

~/tools/awsim-rosbag
❯ source .venv/bin/activate
```

```sh
~/tools/awsim-rosbag
.venv ❯ uv pip install .
```

## 使用方法

### 1. ROSbag記録

- 設定ファイルを準備します（[example_config.yaml](./example_config.yaml)を参考）
- 以下のコマンドを実行します

```sh
~/tools/awsim-rosbag
.venv ❯ awsim-rosbag record --config example_config.yaml
```

### 2. ROSbag変換

- AWSIMとnuScenesのROSbagディレクトリを準備します
- 以下のコマンドを実行します

```sh
~/tools/awsim-rosbag
.venv ❯ awsim-rosbag convert \
    --awsim-bag /path/to/awsim/ \
    --nuscenes-bag /path/to/nuscenes/ \
    --output /path/to/converted_nuscenes/
```

### ヘルプの表示

```sh
~/tools/awsim-rosbag
.venv ❯ awsim-rosbag --help
```

### その他の実行方法

モジュールとして実行することも可能です：

```sh
# ROSbag記録
.venv ❯ python -m awsim_rosbag record --config example_config.yaml

# ROSbag変換
.venv ❯ python -m awsim_rosbag convert \
    --awsim-bag /path/to/awsim/ \
    --nuscenes-bag /path/to/nuscenes/ \
    --output /path/to/converted_nuscenes/
```

## 設定ファイル形式

`config.yaml`の例:

```yaml
# 出力パス (必須)
output_path: "./recorded_bags"

# 記録対象トピック (必須)
topics:
  - /camera/image_raw
  - /lidar/points
  - /imu/data
  - /gnss/fix
  - /vehicle/status

# オプション設定
compression: "file" # 圧縮モード (none, file, message)
storage_id: "sqlite3" # ストレージ形式 (sqlite3, mcap)
max_bag_size: 1024 # 最大バッグサイズ (MB)
max_bag_duration: 300 # 最大記録時間 (秒)
```

## 機能

### recordコマンド

- YAML設定ファイルに基づいてROSbagを記録
- 複数トピックの同時記録
- 圧縮設定対応
- 記録時間・サイズ制限

### convertコマンド

- AWSIMとnuScenesのROSbagを変換・マージ
- db3およびmcap形式対応
- メタデータファイルの保持
- タイムスタンプ形式変換

## 必要な依存関係

- Python 3.8+
- PyYAML 6.0+
- ROS 2 (rosbag2)

## ディレクトリ構造

```text
awsim-rosbag/
├── awsim_rosbag/
│   ├── __init__.py
│   ├── cli.py              # CLIメインエントリーポイント
│   └── commands/
│       ├── __init__.py
│       ├── record.py       # 記録コマンド
│       └── convert.py      # 変換コマンド
├── __main__.py             # Pythonモジュール実行用
├── example_config.yaml     # 設定ファイル例
├── pyproject.toml          # プロジェクト設定
└── README.md               # このファイル
```
