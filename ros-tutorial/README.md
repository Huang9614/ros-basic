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


## `rospkg` 软件包
  
  1. basic requirements
     - 这个包必须有一个符合catkin规范的`package.xml`文件; 这个`package.xml`文件提供有关该软件包的元信息
     - 这个包必须有一个catkin版本的`CMakeLists.txt`文件; 如果它是个Catkin元包的话，则需要有一个`CMakeList.txt`文件的相关样板
     - 每个包必须有自己的目录; 这意味着在同一个目录下不能有嵌套的或者多个软件包存在
       
  2. 创建`catkin`工作区以及软件包
     ```bash
     mkdir -p catkin_ws/src
     cd catkin_ws/src
     catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
     ```
     这里的`catkin_ws` 就是我们的工作区，`catkin_ws/src` 文件夹中又出现了一个文件夹，名为 `beginner_tutorials`，这个文件夹就是ros软件包；在这个包内，可以发现两个文件夹 `/include`, `/src/` 和两个文件： `CMakeLists.txt`, `package.xml`
      
  3. 构建 （`catkin_make`） 创建好的软件包
     ```bash
     cd ~/catkin_ws
     catkin_make
     ```
     我们返回到 `catkin_ws` 工作区中，执行 `catkin_make`，上述命令会构建 `/catkin_make/src` 目录下的所有catkin项目，也就是软件包.
     
     然后在 `catkin_ws` 文件夹中，除了刚刚自己建立的 `catkin_ws/src` 文件夹，还多出来两个文件夹：`catkin_ws/build` and `catkin_ws/devel` ，同时，在 `catkin_ws/src` 文件夹中，多了一个 `CMakeLists.txt` 文件。

     `catkin_ws/build` 目录是构建空间的默认位置，同时cmake和make也是在这里被调用来配置和构建你的软件包。而 `catkin_ws/devel` 目录是开发空间的默认位置, 在安装软件包之前，这里可以存放可执行文件和库。
     
     如果你的源代码不在默认位置（`catkin_ws/src`），比如说存放在了 `catkin_ws/my_src` 中，那可以这样来使用catkin_make:
     `catkin_make --source my_src`

   4. 将工作区 `catkin_ws` 添加到ROS环境中中
      ```bash
      . ~/catkin_ws/devel/setup.bash
      ```
      将这个工作空间添加到ROS环境.

      查看catkin工作区路径
      ```bash
      echo $ROS_PACKAGE_PATH
      ‵``
    5. `rospack find turtlesim` 查找ROS包的路径

        
## ROS节点 `rosnode`
节点实际上只不过是ROS软件包中的一个可执行文件。ROS节点使用ROS客户端库与其他节点通信。节点可以发布或订阅话题，也可以提供或使用服务。ROS客户端库可以让用不同编程语言编写的节点进行相互通信

1. 首先要运行 `roscore`
2. `rosnode list` 显示当前正在运行的ROS节点
3. `rosnode info /rosout` 返回的是指定节点`/rosout`的信息
4. `rosrun turtlesim turtlesim_node` 用包名直接运行软件包内的节点（而不需要知道包的路径）；在这里是运行 `turtlesim` 包中的 `turtlesim_node`
   - `rosrun turtlesim turtlesim_node __name:=my_turtle` 使用重映射参数来改变节点名称
   

## ROS话题 `rostopic`
1. 首先要运行 `roscore`
2. 在新终端中运行 `turtlesim_node`节点： `rosrun turtlesim turtlesim_node`
3. 运行 `turtlesim` 包中的另一个节点，从而实现用键盘控制乌龟运动： `rosrun turtlesim turtle_teleop_key` 
4. `turtlesim_node` 节点和 `turtle_teleop_key` 节点之间是通过一个ROS话题来相互通信的。turtle_teleop_key在话题上发布键盘按下的消息，turtlesim则订阅该话题以接收消息。
5. 查看当前运行了哪些话题： `rostopic list`
   - `rostopic list -h` 查看一下list子命令需要的参数
   - `rostopic list -v` 会列出所有发布和订阅的主题及其类型的详细信息
7. 使用 `rqt_graph` 来显示当前运行的节点和话题：`rosrun rqt_graph rqt_graph`。可以看到，`turtlesim_node`和`turtle_teleop_key`节点正通过一个名为`/turtle1/cmd_vel`的话题来相互通信。
8. `rostopic echo /turtle1/cmd_vel` 显示在某个话题上发布的数据; 再看一下rqt_graph, `rostopic echo` 现在也订阅了turtle1/command_velocity话题。


## ROS消息 `rosmsg`
`rostopic` 话题的通信是通过节点间发送ROS消息实现的。发布者和订阅者必须发送和接收相同类型的消息。这意味着话题的类型是由发布在它上面消息的类型决定的。

1. `rostopic type /turtle1/cmd_vel` 查看发布在话题 `turtle1/cmd_vel` 上的消息的类型。可以看到，在这个话题上，到目前为止，只有一个消息 `geometry_msgs/Twist` 
2. `rosmsg show geometry_msgs/Twist` 使用rosmsg查看消息的详细信息
    -  `rosmsg list` 显示所有话题
3. 在话题 `/turtle1/cmd_vel` 上手动发布类型为 `geometry_msgs/Twist` 的消息：`rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'` 
    - `-l` 让rostopic只发布一条消息，然后退出
    - `--` 这一选项（两个破折号）用来告诉选项解析器，表明之后的参数都不是选项。如果参数前有破折号（-）比如负数，那么这是必需的。
    - `'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' ` 一个 `geometry/Twist` 消息有两个浮点型元素：linear和angular。在本例中，'[2.0, 0.0, 0.0]'表示linear的值为x=2.0, y=0.0, z=0.0，而'[0.0, 0.0, 1.8]'是说angular的值为x=0.0, y=0.0, z=1.8。这些参数实际上使用的是YAML语法
4. `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'` 使用`rostopic pub -r` 来发布源源不断的命令。此时，发现rqt_graph中又多了一个节点，用于在话题 `/turtle1/cmv_vel` 话题上发消息，并且之前的两个节点 `turtlesim_node` 和 `rostopic echo` 也订阅了
5. `rostopic hz /turtle1/pose` 报告turtlesim_node发布/turtle/pose数据速率
6. `rosrun rqt_plot rqt_plot` 在滚动时间图上显示发布到某个话题上的数据


## ROS服务 `rosservice`
服务（Services）是节点之间通讯的另一种方式。服务允许节点发送一个请求（request）并获得一个响应（response）; rosservice可以很容易地通过服务附加到ROS客户端/服务器框架上

  ```bash
  rosservice list         输出活跃服务的信息
  rosservice call         用给定的参数调用服务
  rosservice type         输出服务的类型
  rosservice find         按服务的类型查找服务
  rosservice uri          输出服务的ROSRPC uri
  ```
1. `roscore && rosrun turtlesim turtlesim_node` 运行 `turtlesim` 包中的 `turtlesim_node` 结点
2. `rosservice list` 查看当前活跃的服务列表，按理说应该有11个，9个和 `turtlesim_node` 相关，2个和 `rosout` 结点相关
3. `rosservice type /clear` 查看看 `/clear` 服务的类型；得到的输出为 `std_srvs/Empty`，也就是说服务的类型为空，这表明调用这个服务时不需要参数（即，它在发出请求时不发送数据，在接收响应时也不接收数据）
4. `rosservice call /clear` 调用 `/clear` 服务；因为服务的类型为empty，所以进行无参数调用; 结果是清除了历史运动痕迹
5. `rosservice type /spawn | rossrv show` 查看 `/spawn` 服务的类型，为 `turtlesim/Spawn` 并且查看该类型的详细信息；它将列出服务的请求（输入）和响应（输出）消息的结构，包括消息字段和它们的数据类型
   - `rosservice call /spawn 2 2 0.2 ""` 根据该服务的具体信息可知，调用该服务时，需要输入三个浮点数以及一个字符串：即在给定的位置和角度生成一只新的乌龟。name字段是可选的，这里我们不设具体的名字，让turtlesim自动创建一个。 结果返回一个字符串： `name: Turtle2` 


## ROS参数 `rosparam`
`rosparam` 能让我们在ROS参数服务器（Parameter Server）上存储和操作数据。其使用 `yaml` 标记语言的语法。一般而言，`yaml` 的表述很自然：1是整型，1.0是浮点型，one是字符串，true是布尔型，[1, 2, 3]是整型组成的列表，{a: b, c: d}是字典

```bash
rosparam list 可以看到turtlesim节点在参数服务器上有3个参数用于设定背景颜色
rosparam set /turtlesim/background_r 150  修改背景颜色的红色通道值
rosservice call /clear  调用clear服务使得参数的修改能生效
rosparam get /turtlesim/background_g  查看看参数服务器上其他参数的值。获取背景的绿色通道的值
rosparam get /  显示参数服务器上的所有内容
rosparam dump param.yaml  将所有的参数写入params.yaml文件
rosparam load params.yaml copy_turtle  将yaml文件重载入新的命名空间，例如copy_turtle
```

1. `roscore && rosrun turtlesim turtlesim_node`
2. `rosparam list` 查看参数列表，其中 `/turtlesim` 节点有三个用于改变颜色的参数
3. `rosparam set /turtlesim/background_r 150` 更改其中某个参数的数值，这里将红色通道改成了150
4. `rosservice call /clear` 调用 `/clear` 服务使得上述修改生效
5. `rosparam get /turtlesim/background_g` 查看绿色通道的数值
6. `rosparam dump params.yaml` 将所有的参数写入params.yaml文件
7. `rosparam load params.yaml copy_turtle` 将 `.yaml` 文件重载入新的命名空间：`copy_turtle`
   - `rosparam get /copy_turtle/turtlesim/background_b`
   - `rosparam get /` 此时可以看到有两个一模一样的参数块；只不过一个是以 `copy_turtle` 开始的，一个没有开头；没有开头的这部分是全局参数，没有命名空间前缀
   - `rosparam delete /copy_turtle` 可以将该命名空间删除；此时， `rosparam get /` 将返回和一开始一样的形式

 
## `rqt_console`
  

## `roslaunch`
roslaunch可以用来启动定义在launch（启动）文件中的节点。


### [`.launch` 文件](http://wiki.ros.org/cn/ROS/Tutorials/UsingRqtconsoleRoslaunch)
在ROS（Robot Operating System）中，launch文件是一种用于启动和配置ROS节点的XML格式文件。这些文件被称为“launch文件”是因为它们通常用于启动一个或多个ROS节点，以便配置整个机器人系统的运行环境。

主要作用包括：
- 启动节点（Nodes）： Launch文件可以指定要启动的ROS节点。节点是ROS中的基本执行单元，负责执行特定的任务或功能。
- 设定参数（Parameters）： 你可以使用launch文件来设置ROS参数，这些参数将传递给节点，影响其行为。
- 创建命名空间（Namespaces）： 通过launch文件，你可以为一组节点创建一个独立的命名空间，以避免命名冲突和组织节点。
- 连接节点（Remappings）： Launch文件允许你重新映射节点的输入和输出话题，服务和参数，以便更灵活地配置节点之间的通信。
- 使用条件（Conditions）： 你可以使用条件语句在运行时基于某些条件选择性地启动或配置节点。
- 包含其他launch文件（Inclusion）： Launch文件可以包含其他launch文件，使其更模块化和易于维护。


### 创建 `.launch` 文件
1. `roscd beginner_tutorial && ls` 返回ROS包的文件夹；到目前为止，这个文件夹中包含的内容，还是在 `catkin_create_pkg` 之后形成的
2. `mkdir launch && cd launch` 为 `.launch` 文件创建文件夹；当然，存放launch文件的目录不一定非要命名为launch，事实上都不用非得放在目录中，roslaunch命令会自动查找经过的包并检测可用的启动文件。
3. 创建 `turtlemimic.launch` 文件
4. `roslaunch beginner_tutorials turtlemimic.launch` 使用 `roslaunch` 运行指定包中的指定`.launch`文件

  
## `rosed`
rosed是rosbash套件的一部分。利用它可以直接通过软件包名编辑包中的文件，而无需键入完整路径。比如 `rosed beginner_tutorials [Tap][Tap]` 可以查看这个包有哪些可以编辑的文件，然后选择一个进行编辑，比如 `rosed beginner_tutorials package.xml`


## 创建和构建 `msg` 和 `srv` 文件
`msg`（消息）：`msg` 文件就是文本文件，每行都有一个字段类型和字段名称,用于描述ROS消息的字段。它们用于为不同编程语言编写的消息生成源代码；`srv`（服务）：一个srv文件描述一个服务。它由两部分组成：请求（request）和响应（response）。 

`msg` 文件存放在软件包的 `/msg` 目录下，srv文件则存放在 `/srv` 目录下。

`msg` 文件可以使用的数据类型包括：int8, int16, int32, int64 (以及 uint*)；float32, float64；string；time, duration；其他msg文件；variable-length array[] 和 fixed-length array[C]。ROS中还有一个特殊的数据类型：Header，它含有时间戳和ROS中广泛使用的坐标帧信息。在 `msg` 文件的第一行经常可以看到Header header。

下面是一个 `msg` 文件，它使用了Header，string，和其他另外两个消息的类型；其中 `geometry_msgs` 为ROS包名，可以用 `rospack find geometry_msgs` 找到这个包所在的路径，或者直接 `roscd geometry_msgs` 进入该ROS包； `PoseWithCovariance` 和 `TwistWithCovariance` 其实是两个 `.msg` 文件，定义了两个消息类型。

```bash
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose  
  geometry_msgs/TwistWithCovariance twist
```

`srv` 文件和 `msg` 文件一样，只是它们包含两个部分：请求和响应。这两部分用一条---线隔开，如：

```bash
int64 A
int64 B
---
int64 Sum
```

### 创建 `.msg` 文件 = 自定义话题消息
在ROS的元功能包 `commen_msgs` 中提供了许多不同消息类型的功能包，如： `std_msgs`（标准数据类型），`geometry_msgs`（几何学数据类型），`sensor_msgs`（传感器数据类型）等。但在很多情况下，我们需要针对自己的机器人应用设计特定的消息类型，ROS提供了一套语言无关的消息类型定义方法。

`.msg` 文件是ROS中定义消息类型的文件，一般放在功能包根目录 `beginner_tutorials` 的 `/msg`文件夹中。在功能包的编译过程中，可以使用 `msg` 文件生成不同编程语言使用的代码文件。

1. `roscd beginner_tutorials && ls` 现在可以看到有三个文件夹，包括经过 `catkin_create_pkg` 得到的 `/include` 和 `/src` 文件夹，以及上一节创建的 `/launch` 文件夹
2. `mkdir msg` 创建新文件夹 `/msg`， 专门用来存放自定义的消息类型
3. `echo "int64 num" > msg/Num.msg` 在该文件夹中创建新的消息类型文件 `Num.msg`，里面有一个消息类型：`int64 num`
/-------------接下来需要保证这个文件能够被转换成C++或者Python代码，也就是对 `.msg` 文件进行编译处理----------------/
4. 在 `package.xml` 中添加功能包依赖，即确保它包含以下两行且没有被注释。
```bash
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
5. `roscd beginner_tutorials CMakeLists.txt` 打开CMakeLists.txt文件。
6. 在 `CMakeLists.txt` 中添加编译选项。为已经存在里面的 `find_package(...)` 添加消息生成依赖的功能包： `message_generation` ，这样就能生成消息了。直接将message_generation添加到COMPONENTS列表中即可
7. 在 `CMakeLists.txt` 中找到 `catkin_package()`，在其中添加依赖 `CATKIN_DEPENDS roscpp rospy std_msgs message_runtime`，从而确保导出消息的运行时依赖关系
8. 最后设置需要编译的`.msg`文件。在 `CMakeLists.txt` 的 `Declare ROS messages, services and actions` 模块中找到 `add_message_files()`，添加 `FILES Num.msg`。`add_message_files` 是一个CMake指令，用于告诉ROS构建系统你的软件包包含哪些自定义消息文件，你需要将实际的消息文件名列在 `FILES` 后面；还是在这个模块中，找到 `generate_messages()`，添加 `DEPENDENCIES std_msgs`；手动添加.msg文件后，我们要确保generate_messages()函数被调用，也就是确保CMake知道何时需要重新配置项目。 `generate_messages` 指令用于告诉ROS构建系统，你的软件包包含了自定义消息，并需要生成相应的消息源代码和编译。`DEPENDENCIES` 后面的 `std_msgs` 表示你的自定义消息依赖于 `std_msgs` 这个标准消息包。这是因为你的自定义消息可以使用 `std_msgs` 中定义的基本消息类型作为字段。
9. 回到我们的catkin工作区的根目录，进行编译；运行之后，`beginner_tutorials/msg` 目录中的所有`.msg`文件都将生成对应的代码。我们这里用C++写的，所以会生成C++消息的头文件，并且将生成在 `~/catkin_ws/devel/include/beginner_tutorials/`
```bash
roscd beginner_tutorials
cd ../..
catkin_make
cd 
```
10. `rosmsg show beginner_tutorials/Num` 查看该自定义消息类型；`beginner_tutorials` 是指定义消息的软件包；`Num` 是指消息的名称Num


### 自定义服务数据
与话题消息类似，ROS中的服务数据可以通过`.srv`文件进行语言无关的接口定义，一般放置在功能包根目录下的`/srv`文件夹中。 这个文件包含请求和应答两个数据域，数据域中的内容与话题消息的数据类型相同，只是在请求和应答的描述之间，采用 `----` 分割

1. `roscd beginner_tutorials && ls` 现在可以看到有四个文件夹，包括经过 `catkin_create_pkg` 得到的 `/include` 和 `/src` 文件夹，上一节创建的 `/launch` 文件夹，和刚刚定义的，用来存放自定义消息类型的 `/msg` 文件夹
2. `mkdir srv` 创建新文件夹 `/srv`， 专门用来存放自定义的服务类型
3. `roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv` 这里，我们不自己写 `.srv` 文件了，而是从 `rospy_tutorials` 这个ROS包中，复制一个叫做 `AddTwoInts.srv` 的服务，放到 `beginner_tutorials/srv` 文件夹下
/-------------接下来需要保证这个文件能够被转换成C++或者Python代码，也就是对 `.srv` 文件进行编译处理----------------/
第4步到第7步和在自定义消息时一模一样，需要注意的是，在 `find_package()` 中，`message_generation`对 `msg` 和 `srv` 都适用。
8. 设置需要编译的`.msg`文件。在 `CMakeLists.txt` 的 `Declare ROS messages, services and actions` 模块中找到 `add_service_files()`，添加 `FILES AddTwoInts.srv`。这样，你可以从srv文件定义中生成源代码文件了
9. 回到我们的catkin工作区的根目录，进行编译；运行之后，`beginner_tutorials/srv` 目录中的所有`.srv`文件都将生成对应的代码。我们这里用C++写的，所以会生成C++消息的头文件，并且将生成在 `~/catkin_ws/devel/include/beginner_tutorials/`；和编译 `.msg` 不同的是，此时，会生成三个头文件，包括 `AddTwoInts.h` 以及请求和应答对应的两个头文件
10. `rossrv show beginner_tutorials/AddTwoInts` 查看服务的具体信息


#### CMakeLists.txt中 `find_package()` 的用法说明
`find_package()` 是CMake构建系统中用于在系统中查找和定位外部软件包的命令。它通常与catkin构建系统一起使用，用于在ROS中声明和定位依赖的软件包。

一般来说，`find_package()` 的一般使用方法如下：
```bash
find_package(<Package> [version] [EXACT] [QUIET] [MODULE]
             [REQUIRED] [[COMPONENTS] [components...]])
```
- <Package>: 要查找的软件包的名称。
- version: 软件包的版本要求。
- EXACT: 如果指定了版本，表示要求精确匹配该版本。
- QUIET: 不生成任何错误消息，如果找不到软件包，它将默默地失败。
- MODULE: 指示查找CMake模块而不是配置文件。
- REQUIRED: 表示找不到软件包将导致构建失败。
- COMPONENTS: 指定软件包的组件。COMPONENTS 的使用是ROS构建系统的一种约定，用于指定ROS软件包的具体模块或功能。而对于一些非ROS的软件包，它们可能不需要这种额外的细粒度的声明，因为它们的设计和构建方式可能与ROS软件包有所不同；比如 OpenCV


#### CMakeLists.txt中 `catkin_package()` 的用法说明
`find_package` CMake的一部分，用于找到和配置构建工具和外部库，而 `catkin_package` 是ROS构建系统的一部分，用于告诉 `catkin` 构建系统关于软件包的元信息和依赖关系

- `catkin_package` 用于配置和声明ROS软件包的一些元信息，主要是为了告诉 catkin 构建系统关于当前软件包的一些特定信息。
- 它用于指定ROS软件包的依赖关系，包括运行时所需的ROS软件包，以及其他依赖项。
- `catkin_package` 还用于配置消息生成，特别是当软件包包含自定义消息、服务或行为时

#### `rosmsg` 的相关指令
```bash
rosmsg show     Show message description
rosmsg list     List all messages
rosmsg md5      Display message md5sum
rosmsg package  List messages in a package
rosmsg packages List packages that contain messages
rosmsg -h 查看帮助
```

## 用C++编写发布者和订阅者节点
1. `roscd beginner_tutorials && cd src && ls` 可以看到，当前这个ROS包中，其实什么没有任何节点的，也就是说，`beginner_tutorials/src` 中什么也没有
2. `wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp` 发布者
   - `std_msgs::String msg;` 涉及到 `模板结构体`，`智能指针` 和 结构体中成员的引用
3. `wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/listener/listener.cpp`
   - `void chatterCallback(const std_msgs::String::ConstPtr& msg){...}` 这里的`std_msgs::String`是一个模板结构体，其中有一段 `typedef boost::shared_ptr< ::std_msgs::String_<ContainerAllocator> const> ConstPtr;` 这个`ConstPtr`也能算是结构体的成员？？？结构体的成员不应该用 `.` 访问么？
4. 在 `/beginner_tutorials/CMakeLists.txt` 的 `Build` 模块中，加入下面这段指令；这将创建两个可执行文件talker和listener，默认情况下，它们将被放到软件包目录下的 `devel` 空间中。`catkin_make` 之前，在 `catkin_ws/devel` 文件夹中只有两个文件夹 `python3` 和 `pkgconfig`，和目前的工作无关
   ```bash
   add_executable(talker src/talker.cpp)
   target_link_libraries(talker ${catkin_LIBRARIES})
   add_dependencies(talker beginner_tutorials_generate_messages_cpp) 为可执行目标添加依赖项到消息生成目标，这确保了在使用此包之前生成了该包的消息头

   add_executable(listener src/listener.cpp)
   target_link_libraries(listener ${catkin_LIBRARIES})
   add_dependencies(listener beginner_tutorials_generate_messages_cpp)
   ```
5. 返回 `catkin_ws` 中，执行 `catkin_make`。然后，在文件夹 `catkin_ws/devel` 文件夹中就会出现第三个文件夹 `beginner_tutorials`，其中包含了 `listener` 和 `talker` 两个可执行文件

## 运行及测试发布者和订阅者
想要检验刚刚创建的两个节点是否可用，则只要用`rosrun`来看看效果就行
1. `roscore`
2. `rosrun beginner_tutorials talker`
3. `rosrun beginner_tutorials listener`


## 用C++编写服务和客户端节点
  
## 运行及测试服务和客户端
  
## 录制和回放数据

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
