# 사용법(USAGE)
```
$ colcon build --packages-select gls100_ros2

$ ros2 run gls100_ros2 gls100_node --ros-args -p can_device:=PCAN_USBBUS1 -p can_baudrate:=500000 -p node_id:=10
```
