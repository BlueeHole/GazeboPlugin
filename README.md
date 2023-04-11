# GazeboPlugin
Gazebo插件，可以用键盘控制模型移动，完善中

Ubuntu 20.04 测试通过

## 功能

- [x] 简单的差速模型。使用`w`，`s`控制车辆前后加减速，使用`a`，`d`控制车辆左右旋转。
- [ ] 更真实的阿克曼车型
- [ ] 录制和再播放车辆路径（其实不是很需要，用Rosbag即可）

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
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/your_path/build/
```

最后，启动gazebo。

然后在新窗口直接启动可执行文件`vel`，直接输入`w`，`a`，`s`，`d`即可（由于关闭了shell的conical mode，因此不需要回车）。`vel`会把指令通过gazebo的通信方式发给`libsuv_plugin`供其执行（Gazebo内置通信用的是[Boost的ASIO](https://mmoaay.gitbooks.io/boost-asio-cpp-network-programming-chinese/content/Chapter1.html)）



