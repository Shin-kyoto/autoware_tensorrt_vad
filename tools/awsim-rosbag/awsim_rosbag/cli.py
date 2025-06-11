#!/usr/bin/env python3
"""AWSIM ROSbag CLI メインエントリーポイント."""

import argparse
import sys
from pathlib import Path
from typing import List, Optional

from awsim_rosbag.converter import convert_awsim_to_nuscenes_rosbag
from awsim_rosbag.recorder import record_awsim_rosbag


def create_parser() -> argparse.ArgumentParser:
    """CLIパーサーを作成."""
    parser = argparse.ArgumentParser(
        prog="awsim-rosbag",
        description="AWSIM ROSbag 管理ツール",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用例:
  # AWSIM ROSbagを記録
  awsim-rosbag record --config config.yaml

  # AWSIM to nuScenes ROSbag変換
  awsim-rosbag convert \\
    --awsim-bag /path/to/awsim/ \\
    --nuscenes-bag /path/to/nuscenes/ \\
    --output /path/to/converted_nuscenes/
        """,
    )

    subparsers = parser.add_subparsers(dest="command", help="利用可能なコマンド", metavar="COMMAND")

    # recordサブコマンド
    record_parser = subparsers.add_parser("record", help="AWSIM ROSbagを記録する")
    record_parser.add_argument(
        "--config", type=Path, required=True, help="設定ファイルのパス (YAML形式)"
    )

    # convertサブコマンド
    convert_parser = subparsers.add_parser("convert", help="AWSIM to nuScenes ROSbagを変換する")
    convert_parser.add_argument(
        "--awsim-bag", type=Path, required=True, help="AWSIM ROSbagディレクトリのパス"
    )
    convert_parser.add_argument(
        "--nuscenes-bag",
        type=Path,
        required=True,
        help="nuScenes ROSbagディレクトリのパス",
    )
    convert_parser.add_argument(
        "--output", type=Path, required=True, help="変換後のデータ出力ディレクトリのパス"
    )

    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """メインエントリーポイント."""
    parser = create_parser()
    args = parser.parse_args(argv)

    if not args.command:
        parser.print_help()
        return 1

    try:
        if args.command == "record":
            return record_awsim_rosbag(config_path=args.config)
        elif args.command == "convert":
            return convert_awsim_to_nuscenes_rosbag(
                awsim_bag_path=args.awsim_bag,
                nuscenes_bag_path=args.nuscenes_bag,
                output_path=args.output,
            )
        else:
            print(f"エラー: 不明なコマンド '{args.command}'", file=sys.stderr)
            return 1
    except (FileNotFoundError, ValueError, OSError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
