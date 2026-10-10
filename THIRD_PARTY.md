# 依赖与许可

- `native/goal_dijkstra.cpp`中的堆操作改编自SciPy 1.8.0，原许可完整保留在`native/SCIPY_LICENSE.txt`，说明见`native/THIRD_PARTY.md`。
- `integrations/nav2_costmap_2d`保留Navigation2的许可证和发布器文件中的BSD版权/许可声明。
- NumPy、SciPy、OpenCV、Pillow、PyYAML、psutil与pytest由运行环境安装，其实现和二进制未复制进本分支。
- ROS 2、Nav2及可选OMPL/yaml-cpp由调用方的环境提供。OMPL适配源码是项目已有的通用扩展，默认算法内核构建不编译它。
- `arena_3d_v1`、`arena_evaluation`为原项目使用的算法扩展定义；此副本按算法引用提取，保留接口名称，省略评测入口与默认输入登记。

本导出没有为原项目代码新增或变更授权条款；沿用对应文件的现有许可与版权。
