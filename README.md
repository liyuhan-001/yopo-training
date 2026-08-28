
# You Only Plan Once

Original Paper: [You Only Plan Once: A Learning-Based One-Stage Planner With Guidance Learning](https://ieeexplore.ieee.org/document/10528860)

Improvements and Applications: [YOPOv2-Tracker: An End-to-End Agile Tracking and Navigation Framework from Perception to Action](https://arxiv.org/html/2505.06923v1)

Video of the paper: [YouTube](https://youtu.be/m7u1MYIuIn4), [bilibili](https://www.bilibili.com/video/BV15M4m1d7j5)

Some realworld experiment: [YouTube](https://youtu.be/LHvtbKmTwvE), [bilibili](https://www.bilibili.com/video/BV1jBpve5EkP)

<table>
  <tr>
    <td align="center" width="38.5%"><img src="docs/realworld_1.gif" alt="Fig1" width="100%"></td>
    <td align="center" width="38.5%"><img src="docs/realworld_2.gif" alt="Fig2" width="100%"></td>
    <td align="center" width="23.0%"><img src="docs/platform.gif" alt="Fig3" width="100%"></td>
  </tr>
</table>


### Hardware:
Our drone designed by [@Mioulo](https://github.com/Mioulo) is also open-source. The hardware components are listed in [hardware_list.pdf](hardware/hardware_list.pdf), and the SolidWorks file of carbon fiber frame can be found in [/hardware](hardware/) (complete assembly files are included in the [Release](https://github.com/TJU-Aerial-Robotics/YOPO/releases/tag/hardware)).

## Introduction:
We propose **a learning-based planner for autonomous navigation in obstacle-dense environments** which integrates (i) perception and mapping, (ii) front-end path searching, and (iii) back-end optimization of classical methods into a single network. 

**Learning-based Planner:** Considering the multi-modal nature of the navigation problem and to avoid local minima around initial values, our approach adopts a set of motion primitives as anchor to cover the searching space, and predicts the offsets and scores of primitives for further improvement (like the one-stage object detector YOLO). 

**Training Strategy:** Compared to giving expert demonstrations as labels in imitation learning or exploring by trial-and-error in reinforcement learning, we directly back-propagate the gradients of trajectory costs (e.g. from ESDF) to the weights of network, which is simple, straightforward, accurate, and sequence-independent (free of online simulator interaction or rendering).

<table>
    <tr>
        <td align="center" style="border: none;"><img src="docs/primitive_trajectories.png" alt="Fig1" style="width: 80%;"></td>
        <td align="center" style="border: none;"><img src="docs/predicted_trajectories.png" alt="Fig2" style="width: 80%;"></td>
		<td align="center" style="border: none;"><img src="docs/proposed_guidance_learning.png" alt="Fig3" style="width: 100%;"></td>
    </tr>
    <tr>
        <td align="center" style="border: none;">primitive anchors</td>
        <td align="center" style="border: none;">predicted traj and scores</td>
		<td align="center" style="border: none;">learning method</td>
    </tr>
</table>

### Updates

- **[2026-08-14]** We adopt [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER) as YOPO's trajectory representation, along with several other improvements. Please refer to the [YOPO-MINCO](https://github.com/TJU-Aerial-Robotics/YOPO/tree/YOPO-MINCO) branch. A simple comparison:

<p align="center">
    <img src="docs/yopo_minco.png" alt="compare" />
</p>

- **[2025-6-17]** The code is greatly simplified and refactored in Python/PyTorch. We also replaced the simulator with our CUDA-accelerated randomized environment, which is faster, lightweight, and boundless. For the stable version consistent with our paper, please refer to the [main](https://github.com/TJU-Aerial-Robotics/YOPO/tree/main) branch.

---

## Installation

The project was tested with Ubuntu 20.04 and Jetson Orin/Xavier NX. We assume that you have already installed the necessary dependencies such as CUDA, ROS, and Conda.

**1. Clone the Code**
```
git clone --depth 1 git@github.com:TJU-Aerial-Robotics/YOPO.git
```

**2. Create Virtual Environment**

I specified the version numbers I used currently to avoid future changes; you can remove them.
```
conda create --name yopo python=3.8
conda activate yopo
cd YOPO
pip install -r requirements.txt
```
**3. Build Simulator** 

Build the controller and dynamics simulator
```
conda deactivate
cd Controller
catkin_make
```
Build the environment and sensors simulator (if CUDA errors occur, please refer to [Simulator_Introduction](Simulator/src/readme.md))
```
conda deactivate
cd Simulator
catkin_make
```

## Test the Policy

You can test the policy using pre-trained weights we provide at `YOPO/saved/YOPO_1/epoch50.pth`. 

**1. Start the Controller and Dynamics Simulator** 

For detailed introduction about the controller, please refer to [Controller_Introduction](Controller/src/readme.md)
```
cd Controller
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator_attitude_control.launch
```
**2. Start the Environment and Sensors Simulator**

For detailed introduction about the simulator, please refer to [Simulator_Introduction](Simulator/src/readme.md). Example of a random forest can be found in [random_forest.png](docs/random_forest.png)
```
cd Simulator
source devel/setup.bash
rosrun sensor_simulator sensor_simulator_cuda
```

You can refer to [config.yaml](Simulator/src/config/config.yaml) for modifications of the sensor (e.g., camera and LiDAR parameters) and environment (e.g., scenario type and obstacle density).

**3. Start the YOPO Planner** 

You can refer to [traj_opt.yaml](YOPO/config/traj_opt.yaml) for modification of the flight speed (The given weights are pretrained at 6 m/s and perform smoothly at speeds between 0 - 6 m/s, and more pretrained models are available at [Releases](https://github.com/TJU-Aerial-Robotics/YOPO/releases)).

```
cd YOPO
conda activate yopo
python test_yopo_ros.py --trial=1 --epoch=50
```

**4. Visualization**

Start the RVIZ to visualize the images and trajectory. 
```
cd YOPO
rviz -d yopo.rviz
```

Left: Random Forest (maze_type=5); Right: 3D Perlin (maze_type=1).
<p align="center">
    <img src="docs/new_env.gif" alt="new_env" />
</p>

You can click the `2D Nav Goal` on RVIZ as the goal (the map is infinite so the goal is freely), just like the following GIF ( Flightmare Simulator).

<p align="center">
    <img src="docs/click_in_rviz.gif" alt="click_in_rviz" />
</p>


## Train the Policy
**1. Data Collection** 

For efficiency, we proactively collect dataset (images, states, and map) by randomly resetting the drone's states (positions and orientations). It only takes 1–2 minutes to collect 100,000 samples, and you only need to collect once.
```
cd Simulator
source devel/setup.bash
rosrun sensor_simulator dataset_generator
```
The data will be saved at `./dataset`:
```
YOPO/
├── YOPO/
├── Simulator/
├── Controller/
├── dataset/
```
You can refer to [config.yaml](Simulator/src/config/config.yaml) for modifications of the sampling state, sensor, and environment. Besides, we use random `vel/acc/goal` for data augmentation, and the distribution can be found in [state_samples](docs/state_samples.png)

**2. Train the Policy**
```
cd YOPO/
conda activate yopo
python train_yopo.py
```
It takes less than 1 hour to train on 100,000 samples for 50 epochs on an RTX 3080 GPU and i9-12900K CPU. Besides, we highly recommend binding the process to P-cores  via `taskset -c 1,2,3,4 python train_yopo.py` if your CPU uses a hybrid architecture with P-cores and E-cores. If everything goes well, the training log is as follows:

```
cd YOPO/saved
conda activate yopo
tensorboard --logdir=./
```
<p align="center">
    <img src="docs/train_log.png" alt="train_log" width="100%"/>
</p>

Besides, you can refer to [traj_opt.yaml](YOPO/config/traj_opt.yaml) for modifications of trajectory optimization (e.g. the speed and penalties).


## TensorRT Deployment
We highly recommend using TensorRT for acceleration when flying in real world. It only takes 1ms (with ResNet-14 Backbone) to 5ms (with ResNet-18 Backbone) for inference on NVIDIA Orin NX.

**1. Prepare**
```
conda activate yopo
pip install -U nvidia-tensorrt --index-url https://pypi.ngc.nvidia.com

git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
python setup.py install
```
**2. PyTorch Model to TensorRT**
```
cd YOPO
conda activate yopo
python yopo_trt_transfer.py --trial=1 --epoch=50
```
**3. TensorRT Inference**
```
cd YOPO
conda activate yopo
python test_yopo_ros.py --use_tensorrt=1
```

**4. Adapt to Your Platform**
+ You need to change `env: simulation` at the end of `test_yopo_ros.py` to `env: 435` (this affects the unit of the depth image), and modify the odometry to your own topic (in the NWU frame).

+ Configure your depth camera to match the training configuration (the pre-trained weights use a 16:9 resolution and a 90° FOV; for RealSense, you can set the resolution in ROS-driver file to 480×270).

+ You may want to use the position controller like traditional planners in real flight to make it compatible with your controller. You should change `plan_from_reference: False` to `True` at the end of `test_yopo_ros.py`. You can test the changes in simulation using the position controller: `roslaunch so3_quadrotor_simulator simulator_position_control.launch
`

**5. Generalization**

We use random training scenes, images, and states to enhance generalization. Policy trained with ground truth depth images can be zero-shot transferred to stereo cameras and unseen scenarios:
<p align="center">
    <img src="docs/sim2real.gif" alt="sim2real" />
</p>


## RKNN Deployment
On the RK3566 clip (only 1 TOPS NPU), after deploying with RKNN and INT8 quantization, inference takes only about 20 ms (backbone: ResNet-14). The update of deployment on RK3566 or RK3588 is coming soon.

## Finally
We are still working on improving and refactoring the code to improve the readability, reliability, and efficiency. For any technical issues, please feel free to contact me (lqzx1998@tju.edu.cn) 😀 We are very open and enjoy collaboration!

If you find this work useful or interesting, please kindly give us a star ⭐; If our repository supports your academic projects, please cite our paper. Thank you!

```
@article{YOPO,
  title={You Only Plan Once: A Learning-based One-stage Planner with Guidance Learning},
  author={Lu, Junjie and Zhang, Xuewei and Shen, Hongming and Xu, Liwen and Tian, Bailing},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  publisher={IEEE}
}
```
# YOPO 无人机自主导航科研训练
## 项目简介
本项目为本科科研训练项目，基于 YOPO（You Only Plan Once）无人机自主导航框架开展环境配置、系统部署与仿真实验。
第二阶段主要完成 YOPO 运行环境搭建、Controller 与 Simulator 编译、预训练模型加载以及 RViz 仿真测试。在实验过程中，根据本机 RTX 5060 Laptop GPU 的硬件环境，对 CUDA 及 GPU 架构配置进行了相应调整，并完成多次自主导航与避障测试。
## 一、实验环境
本阶段实验平台配置如下：
| 项目 | 配置 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 LTS |
| ROS | ROS Noetic |
| Python | 3.8.20 |
| Conda 环境 | yopo |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU |
| NVIDIA Driver | 570.211.01 |
| CUDA Toolkit | 12.8 |
| PyTorch | 2.4.1 |
| OpenCV | 4.2.0 |
## 二、项目目录
YOPO 项目主要目录结构如下：
```text
YOPO/
├── Controller/        # 无人机动力学及控制模块
├── Simulator/         # 环境与传感器仿真模块
├── YOPO/              # YOPO 网络及规划模块
├── docs/              # 项目相关文档
├── hardware/          # 硬件相关内容
├── yopo.rviz          # RViz 配置文件
└── README.md
```
三、Python 环境配置
创建 Python 3.8 Conda 环境：
```bash
conda create -n yopo python=3.8
conda activate yopo
```
进入 YOPO Python 模块并安装依赖：
```bash
cd ~/YOPO/YOPO
pip install -r requirements.txt<br/>```
环境安装完成后可通过以下命令进行检查：
```bash
python --version
python -c "import torch; print(torch.__version__);
print(torch.cuda.is_available())"
```
四、Controller 编译
Controller 采用 ROS Catkin 工作空间进行编译。
```bash
conda deactivate
cd ~/YOPO/Controller
catkin_make
```
编译完成后启动无人机动力学与控制节点：
```bash
cd ~/YOPO/Controller
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator_attitude_control.launch
```当终端出现：
```text
TakeOff Done! Ready to Flight...
```
表明 Controller 已正常启动，无人机完成初始化并进入飞行准备状态。
五、Simulator 编译
### 5.1 CUDA 环境
本机 GPU 为 NVIDIA GeForce RTX 5060 Laptop GPU，对应 CUDA Compute Capability 12.0。
系统使用：
```text
CUDA Toolkit 12.8
```
可通过以下命令检查 CUDA 编译工具：
```bash
/usr/local/cuda-12.8/bin/nvcc --version
```
### 5.2 RTX 5060 架构适配
在 Simulator 编译过程中，CMake 自动检测 GPU 架构时无法正确处理 Compute Capability 12.0。
因此修改：
```text
Simulator/src/CMakeLists.txt
```将 CUDA 架构手动设置为：
```cmake
set(ARCH_FLAGS "-gencode arch=compute_120,code=sm_120")
```
### 5.3 Simulator 编译
清除旧编译缓存并指定 CUDA 12.8：
```bash
cd ~/YOPO/Simulator
rm -rf build devel
export PATH=/usr/local/cuda-12.8/bin:$PATH
catkin_make
```
编译完成后启动传感器仿真：
```bash
cd ~/YOPO/Simulator
source devel/setup.bash
export PATH=/usr/local/cuda-12.8/bin:$PATH
rosrun sensor_simulator sensor_simulator_cuda
```
当终端出现：
```text
Simulation Ready!
```
表明随机地图、环境映射以及传感器仿真初始化完成。
六、YOPO 预训练模型运行
打开新的终端并进入 YOPO Conda 环境：
```bash
conda activate yopo
cd ~/YOPO/YOPO
```
运行预训练模型：```bash
python test_yopo_ros.py --trial=1 --epoch=50
```
程序加载：
```text
saved/YOPO_1/epoch50.pth
```
当终端出现：
```text
YOPO Net Node Ready!
```
表明 YOPO 规划节点已成功启动。
七、RViz 可视化与自主避障
打开新的终端：
```bash
cd ~/YOPO
rviz -d yopo.rviz
```
RViz 启动后可观察：
- 无人机模型;
- 环境点云；
- 深度图像；
- YOPO 规划轨迹；
- 无人机实时运动状态。
使用顶部工具栏中的 `2D Nav Goal` 设置目标位置。
设置目标后，YOPO 根据当前环境感知结果生成飞行轨迹，并控制无人机自主向目标位置运动。
终端中出现：
```text
New Goal: (...)
```
表示接收到新的目标点；完成导航后出现:
```text
Arrive!
```
表示无人机已到达目标区域。
本阶段通过设置不同目标点，完成了至少 3 次自主导航与避障测试。
八、系统完整启动流程
系统运行时需要保持 Controller、Simulator、YOPO 和 RViz 同时运行。
推荐启动顺序：
```text
Controller
    ↓
Simulator
    ↓
YOPO Planner
    ↓
RViz
    ↓
设置 2D Nav Goal
    ↓
YOPO 在线规划
    ↓
无人机自主避障
    ↓
到达目标点
```
对应终端如下：
| 终端 | 模块 | 主要命令 |
| --- | --- | --- |
| Terminal 1 | Controller | `roslaunch so3_quadrotor_simulator simulator_attitude_control.launch` |
| Terminal 2 | Simulator | `rosrun sensor_simulator sensor_simulator_cuda` |
| Terminal 3 | YOPO | `python test_yopo_ros.py --trial=1 --epoch=50` |
| Terminal 4 | RViz | `rviz -d yopo.rviz` |
九、环境配置问题与解决记录
### 9.1 CUDA 10.1 与 GCC 版本不兼容
Simulator 初次编译时出现：
```text<br/>unsupported GNU version! gcc versions later than 8 are not supported!
```
检查发现系统原 CUDA Toolkit 为 10.1，无法满足当前硬件及编译环境需求。
后续安装 CUDA Toolkit 12.8，并重新配置 Simulator CUDA 编译环境。
### 9.2 RTX 5060 CUDA 架构无法自动识别
CUDA 12.8 能够检测到本机 GPU 的 Compute Capability 为 12.0，但项目使用的 CMake CUDA 架构检测模块无法自动识别该架构，出现：
```text
Unknown CUDA Architecture Name 12.0
```
在：
```text
Simulator/src/CMakeLists.txt
```中手动加入：
```cmake
set(ARCH_FLAGS "-gencode arch=compute_120,code=sm_120")
```
重新编译后 Simulator 成功生成：
```text<br/>sensor_simulator_cuda<br/>dataset_generator
```完成 RTX 5060 环境下的 Simulator 编译。
### 9.3 PyTorch 与 RTX 5060 架构兼容警告
YOPO 原始依赖环境采用 PyTorch 2.4.1。运行过程中检测到 RTX 5060 `sm_120` 架构兼容警告。
当前实验环境下，YOPO 预训练模型能够正常加载，规划节点可以正常启动，并完成 RViz 仿真导航。
该问题暂未影响本阶段仿真实验，后续将进一步研究新版本 PyTorch 与 RTX 50 系 GPU 的适配方案。
十、第二阶段完成情况
- [x] 获取并检查 YOPO 项目源码
- [x] 创建 Python 3.8 Conda 环境
- [x] 安装 YOPO Python 依赖
- [x] 完成 ROS Noetic 环境检查
- [x] 完成 Controller 编译
- [x] 完成 Controller 运行测试
- [x] 配置 CUDA Toolkit 12.8
- [x] 完成 RTX 5060 `sm_120` 架构适配
- [x] 完成 Simulator 编译
- [x] 完成 Simulator 运行测试
- [x] 加载 YOPO 预训练模型
- [x] 完成 RViz 可视化
- [x] 完成至少 3 次自主避障测试
- [x] 保存系统完整运行视频
十一、阶段总结
本阶段完成了 YOPO 从环境配置、ROS 模块编译到完整仿真运行的部署流程。
针对 RTX 5060 Laptop GPU 与项目原始 CUDA 环境之间的兼容问题，对 CUDA Toolkit 和 Simulator GPU 架构配置进行了调整，最终实现 Controller、Simulator、YOPO Planner 与 RViz 的联合运行。
通过 RViz 设置不同导航目标点，无人机能够利用 YOPO 规划结果进行自主运动，并完成多次自主导航与避障实验。
下一阶段将在当前运行环境基础上进一步开展 YOPO 网络结构、输入输出数据以及规划过程的分析。
