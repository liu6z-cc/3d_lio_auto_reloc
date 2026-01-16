#!/bin/bash
# 性能测试脚本

source ~/catkin_ws/devel/setup.bash

MAP_PATH="/home/ubuntu/catkin_ws/src/algorithm/perception/FASTLIO2_SAM_LC/pcd/okok.pcd"

echo "=== Ultra Fast Map Publisher Benchmark ==="
echo "Testing map: $MAP_PATH"
echo

# 测试1: 原始PCL加载
echo "1. Testing PCL standard loader..."
time rosrun pcl_ros pcd_to_pointcloud $MAP_PATH 0.1 _frame_id:=map cloud_pcd:=/map_test 2>&1 | grep -E "points|time|real" &
PID1=$!
sleep 5
kill $PID1 2>/dev/null

echo
echo "2. Testing Ultra Fast loader (first run, with conversion)..."
time rosrun ultra_fast_map_publisher ultra_fast_map_publisher _map_path:=$MAP_PATH _latch_mode:=true _use_fast_format:=true _auto_convert:=true &
PID2=$!
sleep 10
kill $PID2 2>/dev/null

echo
echo "3. Testing Ultra Fast loader (cached fast format)..."
time rosrun ultra_fast_map_publisher ultra_fast_map_publisher _map_path:=$MAP_PATH _latch_mode:=true _use_fast_format:=true _auto_convert:=false &
PID3=$!
sleep 5
kill $PID3 2>/dev/null

echo
echo "=== Benchmark Complete ==="