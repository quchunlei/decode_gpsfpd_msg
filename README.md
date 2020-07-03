## Decode ROS Msg

- 日期：2020年7月

## 1 编译
```bash
mkdir -p decode_gps_ws/src && cd decode_gps_ws/src
git clone git@192.168.1.20:qcl/decode_msg.git
catkin build 或者 catkin make
source devel/setup.bash 或着 source devel/setup.zsh
```

## 2 使用
### 2.1 修改参数:保存路径和topic name
```c++
  <node pkg ="decode_gps" name="decode_gps" type="decode_gps" output="screen">
    <param name="data_path" value="/home/data/rosbag_data/hryt_0701/hryt_decode_calibration_2020-07-01/"/>
    <param name="gps_topic" value="/gps"/>
  </node>
```
### 2.2 启动程序
```bash
roslaunch decode_gps decode_gps.launch
rosbag play xxx.bag
```