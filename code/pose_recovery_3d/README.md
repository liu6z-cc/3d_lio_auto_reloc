# pose_recovery_3d

3D位姿恢复和自动重定位ROS包

## 功能概述

该包用于3D SLAM系统（如FAST-LIO、LIO-SAM等）的位姿保存与自动恢复：

1. **位姿保存**：定期保存机器人的6DOF位姿（x, y, z, roll, pitch, yaw）
2. **位姿恢复**：在系统重启时自动恢复到上次保存的位姿
3. **自动重定位**：当机器人靠近指定点时自动触发3D重定位

## 节点

### pose_recovery_3d_node

位姿保存节点，订阅3D里程计数据并定期保存到文件。

**订阅话题：**
- `/slam_odom` (nav_msgs/Odometry)：3D SLAM里程计数据

### auto_relocalization_3d_node

自动重定位节点，监听里程计并在需要时调用重定位服务。

**订阅话题：**
- `/slam_odom` (nav_msgs/Odometry)：3D SLAM里程计数据

**服务：**
- `trigger_relocalization` (std_srvs/Trigger)：手动触发重定位

## 使用方法

### 启动

```bash
roslaunch pose_recovery_3d pose_recovery_3d.launch
```

### 参数说明

参见 `param/pose_recovery_3d.yaml` 文件中的注释。

## 与2D版本的区别

| 特性 | 2D (pose_recovery) | 3D (pose_recovery_3d) |
|------|-------------------|----------------------|
| 位姿维度 | x, y, theta | x, y, z, roll, pitch, yaw |
| 订阅话题 | /amcl_pose | /slam_odom |
| 发布/服务 | /initialpose | /slam_reloc |
| 适用系统 | AMCL 2D定位 | FAST-LIO, LIO-SAM等3D SLAM |

## 依赖

- roscpp
- nav_msgs
- geometry_msgs
- tf
- tf2_ros

## 注意事项

1. 确保SLAM系统已正确发布 `/slam_odom` 话题
2. 如需实际调用重定位服务，需根据具体SLAM系统的服务定义修改代码
3. 协方差参数需根据实际定位精度调整
