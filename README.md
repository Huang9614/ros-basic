# ROS1 from 0 

## basics
- rosbash
  -`roscd <pkg-name>` 切换目录（cd）到某个软件包或者软件包集当中
    - 只能切换到那些路径已经包含在ROS_PACKAGE_PATH环境变量中的软件包
    - `echo $ROS_PACKAGE_PATH` 看ROS_PACKAGE_PATH中包含的路径
    - `roscd roscpp/cmake` 也可以切换到一个软件包或软件包集的子目录中
    - `roscd log` 进入存储ROS日志文件的目录
  - `rosls <pkg-name>`
    - 直接按软件包的名称执行 ls 命令（而不必输入绝对路径）
  - `rospack find <pkg-name>` 返回软件包的所在路径

- rospkg
  1. basic requirements
     - 这个包必须有一个符合catkin规范的`package.xml`文件; 这个`package.xml`文件提供有关该软件包的元信息
     - 这个包必须有一个catkin版本的`CMakeLists.txt`文件; 如果它是个Catkin元包的话，则需要有一个`CMakeList.txt`文件的相关样板
     - 每个包必须有自己的目录; 这意味着在同一个目录下不能有嵌套的或者多个软件包存在 
  2. create and catkin_make a initial empty rospkg
     ```bash
     mkdir -p catkin_ws/src
     cd catkin_ws/src
     catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
     ```
     Then you should find new `/include`, `/src/` folders and `CMakeLists.txt`, `package.xml` files in the `~/catkin_ws/src/beginner_tutorial`

     ```bash
     cd ~/catkin_ws
     catkin_make
     ```
     上述命令会构建src目录下的所有catkin项目.
     
     Then you should find new folders `build` and `devel` in the `~/catkin_ws` folder, also a new `CMakeLists.txt` file in the `~/catkin_ws/src` folder.

     `build` 目录是构建空间的默认位置，同时cmake和make也是在这里被调用来配置和构建你的软件包。而`devel` 目录是开发空间的默认位置, 在安装软件包之前，这里可以存放可执行文件和库。
     
     如果你的源代码不在默认位置（catkin_ws/src），比如说存放在了my_src中，那可以这样来使用catkin_make:
     `catkin_make --source my_src`

     ```bash
     . ~/catkin_ws/devel/setup.bash
     ```
     将这个工作空间添加到ROS环境. You can check whether the path of your pakgage is already added by running
     ```bash
     echo $ROS_PACKAGE_PATH
     ```
     
- rosnode
  - `rosnode list` 显示当前正在运行的ROS节点信息
  - `rosnode info /rosout` 返回的是某个指定节点`/rosout`的信息
  - `rosrun turtlesim turtlesim_node` 运行turtlesim包中的turtlesim_node
    - `rosrun turtlesim turtlesim_node __name:=my_turtle` 使用重映射参数来改变节点名称
  - `rosnode ping my_turtle` 使用另外一个rosnode指令，ping，来测试rosnode `my_turtle` 是否正常

- rostopic

  first run `turtle_teleop_key` and `turtlesim_node` in the rospkg `turtlesim`
  ```bash
  roscore
  rosrun turtlesim turtlesim_node
  rosrun turtlesim turtle_teleop_key
  ```
  turtlesim_node节点和turtle_teleop_key节点之间是通过一个ROS话题来相互通信的。turtle_teleop_key在话题上发布键盘按下的消息，turtlesim则订阅该话题以接收消息。使用rqt_graph来显示当前运行的节点和话题

  ```bash
  sudo apt install ros-noetic-rqt
  rosrun rqt_graph rqt_graph
  ```
  可以看到，`turtlesim_node`和`turtle_teleop_key`节点正通过一个名为`/turtle1/cmd_vel`的话题来相互通信。

  - `rostopic -h` 使用帮助选项查看可用的rostopic的子命令; 或者在输入rostopic 之后双击Tab键输出可能的子命令
  - `rostopic echo /turtle1/cmd_vel` 显示在某个话题上发布的数据; 再看一下rqt_graph, rostopic echo现在也订阅了turtle1/command_velocity话题。
  - `rostopic list -h` 查看一下list子命令需要的参数
  - `rostopic list -v` 会列出所有发布和订阅的主题及其类型的详细信息

  话题的通信是通过节点间发送ROS消息实现的。发布者和订阅者必须发送和接收相同类型的消息。这意味着话题的类型是由发布在它上面消息的类型决定的。使用rostopic type命令可以查看发布在话题上的消息的类型。
  - `rostopic type /turtle1/cmd_vel` 查看所发布话题的消息类型
  - `rosmsg show geometry_msgs/Twist` 使用rosmsg查看消息的详细信息
    -  `rosmsg list` show all rosmsgs
  - `rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'` 把数据发布到当前某个正在广播的话题上
    - `-l` 让rostopic只发布一条消息，然后退出
    - `--` 这一选项（两个破折号）用来告诉选项解析器，表明之后的参数都不是选项。如果参数前有破折号（-）比如负数，那么这是必需的。
    - `'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' ` 一个turtlesim/Velocity消息有两个浮点型元素：linear和angular。在本例中，'[2.0, 0.0, 0.0]'表示linear的值为x=2.0, y=0.0, z=0.0，而'[0.0, 0.0, 1.8]'是说angular的值为x=0.0, y=0.0, z=1.8。这些参数实际上使用的是YAML语法
    - `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'` 使用`rostopic pub -r` 来发布源源不断的命令
  - `rostopic hz /turtle1/pose` 报告turtlesim_node发布/turtle/pose数据速率
  - `rosrun rqt_plot rqt_plot` 在滚动时间图上显示发布到某个话题上的数据

- rosservice
  
- rosparam
  
- rqt_console
  
- roslaunch
  
- rosed
  
- 创建和构建msg和srv文件
  
- 用C++编写发布者和订阅者节点
  
- 运行及测试发布者和订阅者
  
- 用C++编写服务和客户端节点
  
- 运行及测试服务和客户端
  
- 录制和回放数据
  
- 从bag文件中读取所需话题的消息的两种方法

## Distinguish
- .yaml file vs .launch file
  1. 在ROS中，.yaml 文件和 .launch 文件都是用于配置和启动ROS节点（ROS节点是ROS中的基本执行单元）的文件
  2. .yaml file
     - 作用： YAML（YAML Ain't Markup Language）是一种轻量级的数据序列化语言，用于描述数据的结构。在ROS中，.yaml 文件通常用于参数配置。
     - 内容： 在 .yaml 文件中，你可以指定ROS节点的参数，例如节点的名称、主题名称、传感器配置等。这允许你在不修改源代码的情况下配置ROS节点的行为。
  3. .launch file
     - 作用： .launch 文件用于启动和配置ROS节点的启动文件。它允许你定义一组节点、参数、重映射等，并在一个文件中启动它们。
     - 内容： 在 .launch 文件中，你可以指定要启动的节点，它们之间的连接，参数设置，重映射（topic、service等的重新映射）等。.launch 文件通常由XML格式编写。
- .xml 文件
  通常用于定义ROS启动文件（launch files）。ROS启动文件是XML格式的文件，用于配置和启动ROS节点的组合。这些文件允许你以一种结构化的方式定义ROS节点的启动参数、节点之间的连接关系、重映射等。see the file <xml_example>
