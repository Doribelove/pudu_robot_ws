# 第三方许可

`goal_dijkstra.cpp` 的 Fibonacci 堆操作改编自 SciPy 1.8.0 的
`scipy/sparse/csgraph/_shortest_path.pyx`。保留该版本的相等代价处理和
CSR 邻居顺序，使目标停止搜索仍输出原来的路径。

原文件：https://github.com/scipy/scipy/blob/v1.8.0/scipy/sparse/csgraph/_shortest_path.pyx

许可见同目录 `SCIPY_LICENSE.txt`。本轮只增加目标停止、float32 权重逐项
提升为 double 和 ctypes 接口，不采用 fast-math。
