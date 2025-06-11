"""ROSbag表現クラス."""

from pathlib import Path

# from typing import Any, Dict, List, Optional

# import yaml


class AWSIMRosbag:
    """AWSIM ROSbagを表現するクラス."""

    def __init__(self, path: Path):
        """AWSIMRosbagを初期化."""
        # topicのmapping情報を保持
        # どんなtopicをrecordするかを保持
        # 保存先を保持
        # recordした後は，topicの情報を保持．各topicごとにlistで保持

        raise NotImplementedError("AWSIMRosbag is not implemented")


class NuScenesRosbag:
    """nuScenes ROSbagを表現するクラス."""

    def __init__(self, path: Path):
        """NuScenesRosbagを初期化."""
        # topicのmapping情報を保持
        # 保存先を保持
        # readした後は，topicの情報を保持．各topicごとにlistで保持

        raise NotImplementedError("NuScenesRosbag is not implemented")
