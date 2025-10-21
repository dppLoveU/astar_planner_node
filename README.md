# astar_planner_node
没有可视化，可直接在命令窗口执行


**第一次运行**

```
# 在项目根目录下执行
mkdir build
cd build
cmake ..
make
./astar_planner
```

**cmake**生成构建系统

cmake .. 

- 读取父目录(`..`)中的`CMakeLists.txt`
- 检查编译器、依赖库
- 生成适合当前系统的构建文件(Makefile)

**为什么需要**：CMake是跨平台的构建系统生成器，它为你生成具体的编译指令。



**make**编译项目

**作用**：实际编译源代码生成可执行文件

**编译过程**：

1. 编译所有`.cpp`文件为`.o`目标文件
2. 链接所有目标文件和库
3. 生成最终的可执行文件`astar_planner`

**后续运行**：如果修改了代码，只需重新make即可

```
cd build
make                 # 只需重新编译
./astar_planner      # 测试修改
```

无需cmake

