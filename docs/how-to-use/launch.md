# How to launch this code

## Prerequisites

- [Build node](./build.md)

## Launch

```bash
source install/setup.bash
```

```bash
ros2 launch autoware_tensorrt_vad vad.launch.xml
```

<details>
<summary>You can get the log like below</summary>

```bash
$ ros2 launch autoware_tensorrt_vad vad.launch.xml log_level:=info
[INFO] [launch]: All log files can be found below /home/autoware/.ros/log/2025-05-27-13-54-06-510476-DPC2305009-965534
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [vad_node-1]: process started with pid [965560]
[vad_node-1] [INFO] [1748321646.630455927] [vad]: VAD Node initialized
```

</details>
