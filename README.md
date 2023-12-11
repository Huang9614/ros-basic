## ros的安装路径
```bash
cd /opt/ros/noetic
```

## `apt-get` 软件包管理库的位置
```bash
cd /etc/apt/sources.list.d
```

## 动态链接库
1. `gazebo` 相关：
```bash
cd /opt/ros/noetic/lib
```

## `std_msgs` 相关
1. `roscd std_msgs` 定位到 `std_msgs` ROS包的位置：`/opt/ros/noetic/share/std_msgs/msg`
   - `cd msg` 中找到 `String.msg` 消息文件
   - `/opt/ros/noetic/include/std_msgs` `String.msg` 消息文件对应的C++文件的路径位置
