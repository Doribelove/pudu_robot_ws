# Nav2变化块发布器源码

这三个文件来自r9使用的继承发布器源码：`CMakeLists.txt`、`src/costmap_2d_publisher.cpp`和`src/delta_snapshot.hpp`。保留Nav2原版权声明与许可，增量编码头文件与Python端解码器配对。

这是匹配Nav2源码包的变更文件集，不是完整Nav2源码树。请在独立的Nav2 Humble源码副本中应用这些文件，按该源码包的依赖与构建流程重建`nav2_costmap_2d_core`，然后由调用方配置相应ROS overlay和库路径。不要覆盖封存库或生产安装。

消息仍以调用方提供的当前地图为输入。仓库只保存变化块编码/传输的实现，不保存实际地图消息或内容。完整ROS复现还需要匹配的Smac、StaticLayer/InflationLayer及明确提供的本地输入；本分支的无现场数据检查不验证完整ROS会话。
