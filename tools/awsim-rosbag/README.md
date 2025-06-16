# AWSIM ROSbag CLI

AWSIM ROSbagの変換を行うCLIツールです。

## セットアップ方法

### 環境のセットアップ

```sh
source /opt/ros/humble/setup.bash
source /opt/autoware/install/setup.bash
```

## 使用方法

### 1. ROSbag記録

- [こちら](https://automotiveaichallenge.github.io/aichallenge-documentation-2025/setup/visible-simulation.html)に従って`./run_evaluation.bash`を実行します
- 以下のコマンドを実行します

```sh
ros2 bag record -o awsim_bag /localization/kinematic_state /map/vector_map /planning/scenario_planning/trajectory /sensing/camera/camera_info /sensing/camera/image_raw /sensing/imu/imu_raw /tf /tf_static
```

### 2. ROSbag変換

- AWSIMとnuScenesのROSbagディレクトリを準備します
- 以下のコマンドを実行します

```sh
~/tools/awsim-rosbag
❯ python awsim_rosbag/convert_awsim_to_ns.py --nuscenes-rosbag ./data/ns_bag/ --input-awsim-rosbag ./data/awsim_bag/ --output-awsim-rosbag ./data/awsim_bag_ns --start-index 120
```
