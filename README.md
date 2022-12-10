# GazeboPlugin
Gazebo插件，可以用键盘控制模型移动，完善中

Ubuntu 20.04 测试通过

## 功能

目前只能实现最简单的绝对位置控制，模拟车辆运动，因为我要用的功能基本就是这么简单，后面再加

## 使用说明

直接cmake->make编译，生成一个库，一个可执行文件，在gazebo模型中添加插件后，运行可执行文件（`vel`），就可以根据指令控制。

```c
suv_plugin.cc —> libsuv_plugin.so
vel_controller.cc -> vel(可执行文件)
```

要使用，首先在包含了某模型的世界文件的`<model>`标签中加这一句：（注意是加在xx.world文件里，而不是model.sdf文件里）

```xml
<plugin name="suv_control" filename="libsuv_plugin.so"/>
```

此外，为了让Gazebo（链接器）能找到，需要运行下述命令

```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/1_ROS/car_controller/build/
```

最后，启动gazebo这个世界。

然后启动可执行文件`vel`，根据`vel_controller.cc`代码中的switch语句输入指令。`vel`会把指令通过gazebo的通信方式发给`libsuv_plugin`供其执行（Gazebo内置通信用的是[Boost的ASIO](https://mmoaay.gitbooks.io/boost-asio-cpp-network-programming-chinese/content/Chapter1.html)）



