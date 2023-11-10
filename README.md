# ROS1 from 0 

## basics
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
