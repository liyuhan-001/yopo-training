yopo‑training项目
项目学习目标：
完成Linux基础命令实操、Python环境调试、Git版本控制与GitHub仓库管理练习。
当前训练进度：
第一阶段任务，完成Ubuntu环境搭建、Linux基础操作练习、仓库初始化与分支管理。
系统版本：
Ubuntu 20.04.6 LTS，内核版本6.14.6‑custom
已完成任务清单：
1. 查看系统版本、磁盘、内存、显卡驱动硬件信息
2. 创建linux_practice完整练习目录结构
3. 编写test_ubuntu.py测试脚本并验证Python运行环境
4. 搭建GitHub仓库，新建dev开发分支
5. 完成3次规范版本提交记录
问题记录：
1. 网页复制代码附带多余html标签，粘贴进nano出现乱码，后续复制代码选用纯文本。
2. 网页版微信调试面板误触F12弹出，刷新页面即可恢复正常。
后续学习计划：
完成YOPO项目的环境配置，并成功运行预训练模型
ROS工作空间搭建‑flightlib编译任务
本次在Ubuntu20.04系统下搭建yopo_ws工作空间，导入flightlib工程源码进行编译。编译初期持续出现catkin_simple依赖相关报错，包括找不到catkin_simple软件包、无法识别catkin_simple()、cs_install()等指令；由于网络环境限制无法正常下载catkin_simple依赖包，最终采用注释CMakeLists.txt内全部catkin‑simple系列调用语句的方式解决报错问题。
执行catkin_make编译后构建进度达到100%，编译成功；通过rospack find flightlib命令验证，ROS能够正常识别flightlib功能包路径，工作空间环境配置完成。
问题记录
报错1：Could not find a package configuration file provided by "catkin_simple"
解决：注释flightlib主目录CMakeLists.txt中include(cmake/catkin.cmake)
报错2：Unknown CMake command "catkin_simple"、"cs_install"
解决：打开flightrender下CMakeLists.txt，注释find_package(catkin_simple REQUIRED)、catkin_simple()、cs_install()、cs_export()全部相关语句。
用到的关键命令
# 进入工作空间
cd ~/yopo_ws
# 编译工作空间
catkin_make
# 刷新ROS环境变量（新开终端必须执行）
source devel/setup.bash
# 验证功能包是否编译成功
rospack find flightlib
# 修改编译配置文件
nano src/flightlib/CMakeLists.txt
nano src/flightrender/CMakeLists.txt
# 清理旧编译缓存
rm -rf build devel
命令说明
1.catkin_make：编译ROS工作空间内所有功能包
2.source devel/setup.bash：刷新环境变量，让ROS识别新编译完成的包，每打开一个新终端都需要执行
3.rospack find flightlib：验证flightlib包编译成功且可被ROS正常识别
4.nano：文本编辑器，修改CMakeLists.txt编译配置文件，注释掉catkin_simple相关代码从而解决依赖报错
5.rm -rf build devel：删除旧的编译缓存文件，修改CMake配置后清理缓存再重新编译
