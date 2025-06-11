"""AWSIM ROSbag記録クラス."""

import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List

import yaml


class AWSIMRosbagRecorder:
    """AWSIM ROSbag記録クラス."""

    def __init__(self):
        """AWSIMRosbagRecorderを初期化."""
        self.config = None

    def load_config(self, config_path: Path) -> Dict[str, Any]:
        """設定ファイルを読み込む."""
        if not config_path.exists():
            raise FileNotFoundError(f"設定ファイルが見つかりません: {config_path}")

        try:
            with open(config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            print(f"設定ファイルを読み込みました: {config_path}")
            return config
        except yaml.YAMLError as e:
            raise ValueError(f"設定ファイルの解析に失敗しました: {e}")

    def validate_config(self, config: Dict[str, Any]) -> None:
        """設定の妥当性を検証."""
        required_keys = ["topics", "output_path"]
        for key in required_keys:
            if key not in config:
                raise ValueError(f"設定ファイルに必須キー '{key}' がありません")

        if not isinstance(config["topics"], list):
            raise ValueError("'topics' はリスト形式である必要があります")

        if not config["topics"]:
            raise ValueError("記録対象のトピックが指定されていません")

    def build_rosbag_command(self, config: Dict[str, Any]) -> List[str]:
        """rosbag2記録コマンドを構築."""
        cmd = ["ros2", "bag", "record"]

        # 出力パス
        output_path = Path(config["output_path"])
        output_path.mkdir(parents=True, exist_ok=True)
        cmd.extend(["-o", str(output_path)])

        # 圧縮設定
        if config.get("compression"):
            cmd.extend(["--compression-mode", config["compression"]])

        # ストレージ設定
        if config.get("storage_id"):
            cmd.extend(["--storage-id", config["storage_id"]])

        # 最大バッグサイズ
        if config.get("max_bag_size"):
            cmd.extend(["--max-bag-size", str(config["max_bag_size"])])

        # 記録時間制限
        if config.get("max_bag_duration"):
            cmd.extend(["--max-bag-duration", str(config["max_bag_duration"])])

        # トピック追加
        for topic in config["topics"]:
            cmd.append(topic)

        return cmd

    def record(self, config_path: Path) -> int:
        """ROSbag記録を実行."""
        print("AWSIM ROSbag記録を開始します...")

        # 設定読み込み
        self.config = self.load_config(config_path)
        self.validate_config(self.config)

        # コマンド構築
        cmd = self.build_rosbag_command(self.config)

        print(f"実行コマンド: {' '.join(cmd)}")
        print(f"出力パス: {self.config['output_path']}")
        print(f"記録対象トピック: {', '.join(self.config['topics'])}")
        print("\n記録を開始します... (Ctrl+Cで停止)")

        # rosbag2コマンド実行
        result = subprocess.run(cmd, check=False)

        if result.returncode == 0:
            print("\nAWSIM ROSbag記録が正常に完了しました")
        else:
            print(f"\nAWSIM ROSbag記録がエラーで終了しました (終了コード: {result.returncode})")

        return result.returncode


def record_awsim_rosbag(config_path: Path) -> int:
    """AWSIM ROSbagを記録する関数."""
    try:
        recorder = AWSIMRosbagRecorder()
        return recorder.record(config_path)
    except KeyboardInterrupt:
        print("\n\nユーザーによって記録が中断されました")
        return 0
    except (FileNotFoundError, ValueError, OSError, yaml.YAMLError) as e:
        print(f"記録中にエラーが発生しました: {e}", file=sys.stderr)
        return 1
