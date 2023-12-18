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

## 命名空间
1. `ros` 这个命名空间的在很多头文件中都进行了定义，分别用来初始化，创建对象等；在使用之前，需要先`#include`。比如说： `#include "ros/ros.h"`，这个头文件中`#include "init.h"`，在 这个 `init.h` 头文件中定义了 `ros` 这个命名空间；又比如，要想使用 `ros::NodeHandle n;`，则需要头文件 `node_hanlde.h`，但是，同样 `#include "ros/ros.h"` 中已经包括了这个头文件了，所以可以直接使用
   - 所以其实 `ros/ros.h` 应该就是专门创建出来，为了形成一个最基本的ros框架构建的；所以在[tutorial](http://wiki.ros.org/cn/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)中，才基本上只需要这一个头文件就行了。
   - 下面是 `ros.h` 头文件中的所有内容
   ```bash
      #ifndef ROSCPP_ROS_H
      #define ROSCPP_ROS_H

      #include "ros/time.h"
      #include "ros/rate.h"
      #include "ros/console.h"
      #include "ros/assert.h"
      
      #include "ros/common.h"
      #include "ros/types.h"
      #include "ros/node_handle.h"
      #include "ros/publisher.h"
      #include "ros/single_subscriber_publisher.h"
      #include "ros/service_server.h"
      #include "ros/subscriber.h"
      #include "ros/service.h"
      #include "ros/init.h"
      #include "ros/master.h"
      #include "ros/this_node.h"
      #include "ros/param.h"
      #include "ros/topic.h"
      #include "ros/names.h"

      #endif
      ```
  
## 源码相关
1. 在C++中，通常情况下，你只需分享头文件（.h）和编译后的二进制文件（共享库或静态库），而不必分享源代码（.cpp文件）。这有助于保护你的源代码，同时允许其他人使用你的库；所以在下载的ROS文件夹中，只能找到`.h`文件，而找不到`.cpp`文件
   - 如何查看ROS相关的`.cpp` 文件？？？
