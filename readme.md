# 🚀3D_LIO_AYTO_RELOC
by：liu6z
Applicable Scenarios：ROS Melodic下的大规模点云地图处理与自动重定位

## 环境适配
- system：ubuntu-18.04
- ROS Version：Melodic
- Python Version：2.7 

# ---------------------------工程配置--------------------------------

## ------UltraFastMapPublisher功能包介绍------
UltraFastMapPublisher 是针对 ROS 系统大规模点云地图发布场景的高性能优化组件，核心解决原生 pcl_ros/pcd_to_pointcloud 节点在处理千万级点云时 IO 效率低、CPU 利用率不足、无状态反馈的问题。组件基于内存映射、多线程并行解析、自定义紧凑格式等底层优化，实现点云加载速度 3-10 倍提升，同时保留原生节点核心功能并扩展监控、缓存、鲁棒性等特性。

1. 底层读取加速
新增 MemoryMappedFile 类封装内存映射能力，替代传统文件流读取方式，跳过 “磁盘→内核缓冲区→用户缓冲区” 的拷贝步骤，直接从内存地址访问文件数据，大幅降低大文件读取的耗时。
2. 自定义紧凑格式Fast-Format
针对 PCD 文件文本头冗余、解析繁琐的问题，设计轻量二进制格式：仅保留点云数量、坐标范围等核心头部信息，数据区只存储 X/Y/Z 坐标（剔除 PCD 无效的第四维数据），文件体积减少 15%-20%，解析逻辑简化 80%。
3. 智能缓存与自动转换
自动缓存转换后的 Fast Format 文件，通过文件修改时间（mtime）判断是否需要重新转换，避免频繁重启节点时重复解析 PCD；开启自动转换后，无紧凑格式文件时会自动生成，失败则回退到 PCD 加载，不影响核心功能。
4. 可视化进度与统计
新增 ConsoleProgressBar 进度回调机制，实时输出加载阶段、进度百分比、已处理点数；额外发布 /map_stats 话题，输出点云数量、加载耗时、坐标范围等统计信息，方便调试和监控。
<div align="center">
  <img src="https://raw.githubusercontent.com/liu6z-cc/3d_lio_auto_reloc/main/Flowchart/UltraFastMapPublisher.png" 
       alt="UltraFastMapPublisher工作流程图" 
       width="600"
       style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);">
  <br>
  <em>UltraFastMapPublisher节点工作流程图</em>
</div>
### ---使用事项---
注意事项：必需二进制pcd文件为输入源
python convert_pcd_to_binary.py [输入文件] [输出文件]

## ------lio_localization功能包介绍------
更新之后的global_localization_old.py为主要实现脚本

1. FAISS 索引加速：旧版全局地图配准时需遍历全部点云，大规模地图下效率极低；新版引入 FAISS 近邻搜索库，构建全局地图的 L2 索引，通过 range_search 快速查询指定半径内的点云，配准效率提升 10 倍以上。
2. 地图缓存机制：新增本地缓存功能，首次加载地图后会将点云数据和 FAISS 索引保存到 ~/.lio_map_cache 目录，后续启动直接加载缓存文件，冷启动速度提升 80%+。
3. 异步处理框架：地图解析、点云构建、索引创建等耗时操作全部异步执行，避免主线程阻塞，节点启动更快，响应更及时。
<div align="center">
  <img src="https://raw.githubusercontent.com/liu6z-cc/3d_lio_auto_reloc/main/Flowchart/lio_localization.png" 
       alt="lio_localization工作流程图" 
       width="600"
       style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);">
  <br>
  <em>lio_localization节点工作流程图</em>
</div>
### ---注意---
1. 首次启动节点会构建 FAISS 索引，耗时稍长，后续启动加载缓存会大幅提速
2. 首次运行之后，会在/home/ubuntu/.lio_map_cache生成相应的缓存文件，若地图更新需要删除lio_ma_cache文件夹
3. 定位阈值需根据实际场景微调，过低可能导致定位漂移，过高会降低成功率
REGISTRATION_RADIUS： 以当前位姿为中心，FAISS 查询周边点云的半径（单位：米）
DISPLAY_RADIUS：      可视化用查询半径（仅影响 RViz 显示，不影响定位核心逻辑）
position_tolerance：  重定位验证的位置容差（单位：米）：当前位姿与目标位姿的距离误差小于该值视为重定位成功
yaw_tolerance：       重定位验证的角度容差（单位：弧度）：当前航向与目标航向的误差小于该值视为成功

## ------pose_recovery_3d功能包介绍------
### pose_recovery_3d_node
实现位姿记录，监听机器人的全局位姿，定时自动保存到 param/下的两个JSON 文件（缓冲区交替写入），系统重启时加载历史位姿；仅在重定位完成后才开始保存，避免保存错误位姿。
<div align="center">
  <img src="https://raw.githubusercontent.com/liu6z-cc/3d_lio_auto_reloc/main/Flowchart/pose_recovery_3d_node.png" 
       alt="pose_recovery_3d_node工作流程图" 
       width="600"
       style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);">
  <br>
  <em>pose_recovery_3d_node节点工作流程图</em>
</div>

### auto_relocalization_3d_node
实现自动重定位与验证，系统重启后自动加载历史位姿，发布/initialpose触发 SLAM 重定位，验证重定位（position_tolerance + yaw_tolerance），重定位成功后通知位姿保存节点开始工作；同时支持目标点触发重定位、手动服务触发重定位。
<div align="center">
  <img src="https://raw.githubusercontent.com/liu6z-cc/3d_lio_auto_reloc/main/Flowchart/auto_relocalization_3d_node.png" 
       alt="auto_relocalization_3d_node工作流程图" 
       width="600"
       style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1);">
  <br>
  <em>auto_relocalization_3d_node节点工作流程图</em>
</div>

#### ---使用事项---
第一次加载地图之后，需手动拉起2D_estimate进行重定位，成功之后会自动在param下建立JSON并写入坐标，后续使用无需手动拉起。

# --------------------------环境与安装-------------------------------



## 所需三方库
- numpy = 1.16.6
- ros-numpy
- open3d = 0.9.0
- faiss_cpu-1.6.1
- cmake = 3.20.6

## ------具体安装步骤------
### 1. cmake安装
升级cmake适配faiss库，原生cmake版本过低
sudo tar -zxvf cmake-3.20.6-linux-x86_64.tar.gz -C /usr/local/

PATH=/usr/local/cmake-3.20.6-linux-x86_64/bin:$PATH  # 进入bashrc输入以下内容配置环境变量

cmake --version # 验证版本（输出3.20.6则成功）

### 2. numpy安装
sudo apt update
sudo apt install python2-pip
pip2 install numpy==1.16.6 --user  # 1.16.6是Python2兼容的最后稳定版

### 3. ros-numpy安装
cd ~/catkin_ws/src
git clone https://github.com/eric-wieser/ros_numpy.git
cd ~/catkin_ws
catkin_make 
source devel/setup.bash

### 4. open3d安装
pip2 install open3d==0.9.0 --user

### 5. faiss安装

pip2 install faiss_cpu-1.6.1-cp27-cp27mu-manylinux1_x86_64.whl --user  

sudo apt install libopenblas-dev liblapack-dev libgfortran3  # 补充库文件

## 安装验证

python2  # 启动python2环境
import numpy  
print("numpy版本：", numpy.__version__)
import faiss
print("FAISS版本：", faiss.__version__)
import ros_numpy
print("ros_numpy导入成功！")
