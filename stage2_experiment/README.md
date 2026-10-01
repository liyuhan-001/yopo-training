# YOPO 无人机自主导航科研训练——第二阶段报告

## 项目简介

本项目为本科科研训练项目，基于 YOPO（You Only Plan Once）无人机自主导航框架开展环境配置、系统部署与仿真实验。

第二阶段主要完成 YOPO 运行环境搭建、Controller 与 Simulator 编译、预训练模型加载和 RViz 仿真测试。针对本机 NVIDIA GeForce RTX 5060 Laptop GPU，对 CUDA Toolkit 和 GPU 架构配置进行了适配，并完成多次自主导航与避障测试。

## 1. 实验环境

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic |
| Python | 3.8.20 |
| Conda 环境 | `yopo` |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU |
| NVIDIA Driver | 570.211.01 |
| CUDA Toolkit | 12.8 |
| PyTorch | 2.4.1 |
| OpenCV | 4.2.0 |

## 2. 项目目录

YOPO 项目主要目录结构如下：

```text
YOPO/
├── Controller/    # 无人机动力学与控制模块
├── Simulator/     # 环境与传感器仿真模块
├── YOPO/          # YOPO 网络与规划模块
├── docs/          # 项目相关文档
├── hardware/      # 硬件相关内容
├── yopo.rviz      # RViz 配置文件
└── README.md
```

## 3. Python 环境配置

创建 Python 3.8 Conda 环境并激活：

```bash
conda create -n yopo python=3.8
conda activate yopo
```

进入 YOPO Python 模块并安装依赖：

```bash
cd ~/YOPO/YOPO
pip install -r requirements.txt
```

检查 Python、PyTorch 版本及 CUDA 是否可用：

```bash
python --version
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

## 4. Controller 编译与启动

使用 ROS Catkin 工作空间编译 Controller：

```bash
conda deactivate
cd ~/YOPO/Controller
catkin_make
```

启动无人机动力学与控制节点：

```bash
cd ~/YOPO/Controller
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator_attitude_control.launch
```

终端出现以下信息时，表示 Controller 已启动并进入飞行准备状态：

```text
TakeOff Done! Ready to Flight...
```

## 5. Simulator 编译与启动

### 5.1 CUDA 环境检查

本机 GPU 的 Compute Capability 为 12.0，使用 CUDA Toolkit 12.8。可通过以下命令检查 CUDA 编译工具：

```bash
/usr/local/cuda-12.8/bin/nvcc --version
```

### 5.2 RTX 5060 架构适配

Simulator 编译过程中，项目原有的 CMake CUDA 架构检测未能自动识别 RTX 5060 的 `sm_120` 架构。因此，在 `Simulator/src/CMakeLists.txt` 中添加了以下架构配置：

```cmake
set(ARCH_FLAGS "-gencode arch=compute_120,code=sm_120")
```

### 5.3 Simulator 编译

清理旧编译目录并使用 CUDA Toolkit 12.8 重新编译：

```bash
cd ~/YOPO/Simulator
rm -rf build devel
export PATH=/usr/local/cuda-12.8/bin:$PATH
catkin_make
```

> 注意：`rm -rf build devel` 会删除 Simulator 目录下的编译生成目录。这里记录的是本阶段使用的编译步骤；重新操作前应确认目录中没有需要保留的内容。

编译完成后启动传感器仿真：

```bash
cd ~/YOPO/Simulator
source devel/setup.bash
export PATH=/usr/local/cuda-12.8/bin:$PATH
rosrun sensor_simulator sensor_simulator_cuda
```

终端出现以下信息时，表示仿真环境和传感器已启动：

```text
Simulation Ready!
```

## 6. YOPO 预训练模型运行

激活 YOPO Conda 环境并运行 ROS 测试程序：

```bash
conda activate yopo
cd ~/YOPO/YOPO
python test_yopo_ros.py --trial=1 --epoch=50
```

程序加载的预训练模型为：

```text
saved/YOPO_1/epoch50.pth
```

终端出现以下信息时，表示 YOPO 规划节点已启动：

```text
YOPO Net Node Ready!
```

## 7. RViz 可视化与自主避障测试

启动 RViz：

```bash
cd ~/YOPO
rviz -d yopo.rviz
```

RViz 中可以查看无人机模型、环境点云、深度图像和 YOPO 规划轨迹。使用工具栏中的 **2D Nav Goal** 设置目标位置后，YOPO 根据当前环境生成轨迹并控制无人机移动。

终端中出现 `New Goal: (...)` 表示系统收到新的目标；导航完成后出现 `Arrive!` 表示无人机到达目标区域。

本阶段通过设置不同目标位置，完成了至少 3 次自主导航与避障测试。

## 8. 系统启动流程

系统运行时需要同时启动 Controller、Simulator、YOPO 和 RViz。建议按下表在不同终端中运行：

| 终端 | 模块 | 主要命令 |
|---|---|---|
| Terminal 1 | Controller | `roslaunch so3_quadrotor_simulator simulator_attitude_control.launch` |
| Terminal 2 | Simulator | `rosrun sensor_simulator sensor_simulator_cuda` |
| Terminal 3 | YOPO | `python test_yopo_ros.py --trial=1 --epoch=50` |
| Terminal 4 | RViz | `rviz -d yopo.rviz` |

完整流程为：

```text
启动 Controller
      ↓
启动 Simulator
      ↓
启动 YOPO Planner
      ↓
启动 RViz
      ↓
使用 2D Nav Goal 设置目标
      ↓
YOPO 在线规划并控制无人机自主避障
      ↓
到达目标区域
```

## 9. 环境配置问题及解决方法

### 9.1 CUDA 10.1 与当前编译环境不兼容

初次编译 Simulator 时，系统原有 CUDA Toolkit 10.1 未满足本机硬件及当前编译环境的需求。随后安装 CUDA Toolkit 12.8，并重新配置 Simulator 的 CUDA 编译环境。

### 9.2 RTX 5060 的 CUDA 架构未被自动识别

CUDA Toolkit 12.8 能够识别本机 GPU 的 Compute Capability 12.0，但项目原有的 CMake CUDA 架构检测模块未能自动识别该架构，出现以下提示：

```text
Unknown CUDA Architecture Name 12.0
```

在 `Simulator/src/CMakeLists.txt` 中手动指定 `compute_120` 和 `sm_120` 后，Simulator 成功编译。

### 9.3 PyTorch 与 RTX 5060 架构兼容性提示

运行过程中，PyTorch 2.4.1 对 RTX 5060 的 `sm_120` 架构发出兼容性提示。本阶段预训练模型仍能正常加载，YOPO 规划节点可以启动，RViz 仿真导航与避障测试也能够完成。后续可进一步评估较新版本 PyTorch 对该 GPU 架构的支持情况。

## 10. 第二阶段完成情况

- [x] 获取并检查 YOPO 项目源码
- [x] 创建 Python 3.8 Conda 环境
- [x] 安装 YOPO Python 依赖
- [x] 完成 ROS Noetic 环境检查
- [x] 完成 Controller 编译与运行测试
- [x] 配置 CUDA Toolkit 12.8
- [x] 完成 RTX 5060 `sm_120` 架构适配
- [x] 完成 Simulator 编译与运行测试
- [x] 加载 YOPO 预训练模型
- [x] 完成 RViz 可视化
- [x] 完成至少 3 次自主导航与避障测试
- [x] 保存系统完整运行视频

## 阶段总结

本阶段完成了 YOPO 从运行环境配置、ROS 模块编译到仿真运行的部署流程。针对 RTX 5060 Laptop GPU 与项目原有编译配置之间的兼容问题，调整了 CUDA Toolkit 和 Simulator 的 GPU 架构配置。

通过 RViz 设置不同导航目标，YOPO 能够根据环境生成规划轨迹并控制无人机自主移动、避障。本阶段完成了至少 3 次自主导航与避障测试，为后续分析 YOPO 网络输入输出和规划过程打下了基础。
