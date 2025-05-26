# How to build this code

## source

- Please source `setup.bash` in ROS 2 and autoware.

```bash
# example
source /opt/ros/humble/setup.bash
source /home/tier4/pilot-auto.xx1/install/setup.bash
```

## build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_tensorrt_vad
```
