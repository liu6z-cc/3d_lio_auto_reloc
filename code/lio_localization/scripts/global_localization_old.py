#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import threading
import time
import os
import sys
import numpy as np
import tf
import tf2_ros
import rospy
import ros_numpy
import open3d as o3d
import faiss  # 离线安装的FAISS（Python2版本）
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

# ===================== 全局变量（修复线程安全）=====================
global_map = None
global_map_np = None
global_map_normals = None
global_map_gravity = None
global_map_index = None  # FAISS索引
map_bounds = (np.zeros(3), np.zeros(3))
initialized = False  # 定位状态：是否初始化成功
T_map_to_odom = np.eye(4)  # 地图到里程计的变换（核心对齐参数）
cur_odom = None
cur_scan = None
submap_publish_lock = threading.Lock()
tf_broadcaster = None  # TF广播器
index_ready = False     # FAISS索引是否就绪
map_ready = False       # 地图数据是否解析完成
scan_valid = False      # 扫描数据是否有效
new_initial_pose = None  # 存储新的初始位姿

# 线程锁（核心修复：解决多线程数据竞争）
global_lock = threading.Lock()

# ===================== ROS发布器 =====================
pub_pc_in_map = None
pub_submap = None
pub_map_to_odom = None

# ===================== 配置参数（关键补丁：适配探维360）=====================
# 定位阈值（补丁1：适配探维360的实际匹配分数）
LOCALIZATION_TH_init = 0.1    # 核心补丁：大幅降低初始阈值（探维360匹配分数低）
LOCALIZATION_TH_max = 0.7     # 补丁：适配探维360
LOCALIZATION_TH = 0.5         # 补丁：适配探维360
# 点云下采样参数
MAP_VOXEL_SIZE = 0.05
SCAN_VOXEL_SIZE = 0.01
# 频率/视野参数（补丁2：适配探维360 360度激光）
FREQ_LOCALIZATION = 1
FOV = 6.28  # 补丁：改为2π，适配360度激光
FOV_FAR = 50
FOV_FAR_edge = 2.0
FOV_FAR_offset = 2.0
# 配准/可视化半径（补丁3：增大FAISS查询半径，适配探维360点云密度）
DISPLAY_RADIUS = 30.0  # 补丁：从15→30，保证足够的配准点
REGISTRATION_RADIUS = 10.0  # 补丁：从5→10，适配探维360
# 坐标系/TF参数
PUBLISH_TF = True
FRAME_ID_MAP = "map"
FRAME_ID_BASE = "base_link"
# 重试/超时参数
POLL_TIMEOUT = 10
MAX_ATTEMPTS = 100  # 重新初始化最大重试次数
POSE_WAIT_TIMEOUT = 5.0
ICP_TIMEOUT = 2.0  # ICP超时时间（秒）
# 2D Pose Estimate坐标系修正（补丁4：适配探维360坐标系）
INIT_POSE_ROT_CORRECTION = 0.0  # 补丁：取消额外旋转（探维360无需修正）

# ===================== 缓存配置 =====================
CACHE_ENABLED = True
CACHE_DIR = os.path.expanduser("~/.lio_map_cache")
CACHE_POINT_FILE = "global_map_points.npz"
CACHE_INDEX_FILE = "global_map_index.faiss"
CACHE_POINT_COUNT = 49048115


def create_cache_dir():
    if not os.path.exists(CACHE_DIR):
        try:
            os.makedirs(CACHE_DIR)
            rospy.loginfo("[DEBUG] Cache directory created: %s" % CACHE_DIR)
        except Exception as e:
            rospy.logerr("[DEBUG] Failed to create cache directory: %s" % str(e))
            return False
    return True


def load_map_from_cache():
    global global_map_np, global_map_normals, global_map_gravity
    cache_path = os.path.join(CACHE_DIR, CACHE_POINT_FILE)
    
    if not CACHE_ENABLED or not os.path.exists(cache_path):
        rospy.loginfo("[DEBUG] Point cache not found: %s" % cache_path)
        return False
    
    try:
        rospy.loginfo("[DEBUG] Loading points from cache (fast mode)...")
        load_start = time.time()
        
        with np.load(cache_path, allow_pickle=True) as data:
            global_map_np = data['points'].astype(np.float32).copy()
            global_map_normals = data.get('normals', np.array([]))
            global_map_gravity = data.get('gravity', np.array([]))
        
        if len(global_map_np) != CACHE_POINT_COUNT:
            rospy.logwarn("[DEBUG] Point cache invalid: count mismatch")
            global_map_np = None
            return False
        
        rospy.loginfo("[DEBUG] Points loaded in %.3fs (count: %d)" % (time.time()-load_start, len(global_map_np)))
        return True
    except Exception as e:
        rospy.logerr("[DEBUG] Failed to load point cache: %s" % str(e))
        global_map_np = None
        return False


def load_index_from_cache():
    global global_map_index
    cache_path = os.path.join(CACHE_DIR, CACHE_INDEX_FILE)
    
    if not CACHE_ENABLED or not os.path.exists(cache_path):
        rospy.loginfo("[DEBUG] Index cache not found: %s" % cache_path)
        return False
    
    try:
        rospy.loginfo("[DEBUG] Loading FAISS index (fast mode)...")
        load_start = time.time()
        
        global_map_index = faiss.read_index(cache_path)
        rospy.loginfo("[DEBUG] FAISS index loaded in %.3fs" % (time.time()-load_start))
        return True
    except Exception as e:
        rospy.logerr("[DEBUG] Failed to load index cache: %s" % str(e))
        return False


def save_map_to_cache():
    if not CACHE_ENABLED or global_map_np is None:
        return
    
    if not create_cache_dir():
        return
    
    cache_path = os.path.join(CACHE_DIR, CACHE_POINT_FILE)
    try:
        rospy.loginfo("[DEBUG] Saving points to cache (async)...")
        save_start = time.time()
        
        normals_data = global_map_normals if (global_map_normals is not None and len(global_map_normals) > 0) else []
        gravity_data = global_map_gravity if (global_map_gravity is not None and len(global_map_gravity) > 0) else []
        
        np.savez_compressed(
            cache_path,
            points=global_map_np,
            normals=normals_data,
            gravity=gravity_data
        )
        
        rospy.loginfo("[DEBUG] Points saved in %.3fs" % (time.time()-save_start))
    except Exception as e:
        rospy.logerr("[DEBUG] Failed to save point cache: %s" % str(e))


def save_index_to_cache():
    if not CACHE_ENABLED or global_map_index is None:
        return
    
    if not create_cache_dir():
        return
    
    cache_path = os.path.join(CACHE_DIR, CACHE_INDEX_FILE)
    try:
        rospy.loginfo("[DEBUG] Saving FAISS index to cache (async)...")
        save_start = time.time()
        
        faiss.write_index(global_map_index, cache_path)
        rospy.loginfo("[DEBUG] FAISS index saved in %.3fs" % (time.time()-save_start))
    except Exception as e:
        rospy.logerr("[DEBUG] Failed to save index cache: %s" % str(e))


def build_faiss_index_async():
    global global_map_index, index_ready
    rospy.loginfo("[DEBUG] Building FAISS index (fast async)...")
    build_start = time.time()
    
    if load_index_from_cache():
        index_ready = True
        rospy.loginfo("[DEBUG] FAISS index ready in %.3fs" % (time.time()-build_start))
        return
    
    wait_count = 0
    while (global_map_np is None or len(global_map_np) == 0) and not rospy.is_shutdown() and wait_count < 50:
        time.sleep(0.1)
        wait_count += 1
    
    if global_map_np is None or len(global_map_np) == 0:
        rospy.logerr("[DEBUG] No points to build FAISS index")
        return
    
    try:
        global_map_index = faiss.IndexFlatL2(3)
        global_map_index.add(global_map_np)
        
        index_ready = True
        rospy.loginfo("[DEBUG] FAISS index built in %.3fs (size: %d)" % (time.time()-build_start, global_map_index.ntotal))
        
        t = threading.Thread(target=save_index_to_cache)
        t.daemon = True
        t.start()
    except Exception as e:
        rospy.logerr("[DEBUG] FAISS build error: %s" % str(e))
        index_ready = False


def build_open3d_pointcloud_async():
    global global_map
    rospy.loginfo("[DEBUG] Building Open3D pointcloud (async, non-blocking)...")
    build_start = time.time()
    
    while (global_map_np is None or len(global_map_np) == 0) and not rospy.is_shutdown():
        time.sleep(0.1)
    
    if global_map_np is None:
        rospy.logerr("[DEBUG] No points to build Open3D pointcloud")
        return
    
    try:
        global_map = o3d.geometry.PointCloud()
        global_map.points = o3d.utility.Vector3dVector(global_map_np)
        
        if global_map_normals is not None and len(global_map_normals) > 0:
            global_map.normals = o3d.utility.Vector3dVector(global_map_normals)
        
        rospy.loginfo("[DEBUG] Open3D pointcloud built in %.3fs" % (time.time()-build_start))
    except Exception as e:
        rospy.logerr("[DEBUG] Open3D build error: %s" % str(e))


def query_faiss_ball_point(center, radius):
    if global_map_index is None or not index_ready or global_map_np is None:
        return np.array([], dtype=np.int64)
    
    try:
        center = center.astype(np.float32).reshape(1, 3)
        radius2 = radius * radius
        lims, D, I = global_map_index.range_search(center, radius2)
        
        max_points = 500000
        if len(I) > max_points:
            I = I[:max_points]
        return I
    except Exception as e:
        rospy.logerr("[DEBUG] FAISS query error: %s" % str(e))
        return np.array([], dtype=np.int64)


def fix_rotation_matrix(mat):
    """修复旋转矩阵并适配坐标系"""
    mat = np.copy(mat)
    rot_correction = tf.transformations.euler_matrix(0, 0, INIT_POSE_ROT_CORRECTION)
    mat[:3, :3] = np.dot(mat[:3, :3], rot_correction[:3, :3])
    
    # 正交化修正：防止旋转矩阵非正交
    u, s, vh = np.linalg.svd(mat[:3, :3])
    mat[:3, :3] = np.dot(u, vh)
    
    return mat


def initial_pose_to_mat(pose_msg):
    """核心修复：专门处理/initialpose的2D位姿转换"""
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    
    # 1. 基础位姿转换
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    rot = tf.transformations.quaternion_matrix(q)
    trans = tf.transformations.translation_matrix([position.x, position.y, position.z])
    mat = np.dot(trans, rot)
    
    # 2. 2D Pose Estimate坐标系修正（匹配激光/地图方向）
    rot_correction = tf.transformations.euler_matrix(0, 0, INIT_POSE_ROT_CORRECTION)
    mat[:3, :3] = np.dot(mat[:3, :3], rot_correction[:3, :3])
    
    # 3. 正交化修正
    u, s, vh = np.linalg.svd(mat[:3, :3])
    mat[:3, :3] = np.dot(u, vh)
    
    return mat


def pose_to_mat(pose_msg):
    """通用位姿转矩阵（适配Odometry/PoseWithCovarianceStamped）"""
    if isinstance(pose_msg, Odometry):
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation
    elif isinstance(pose_msg, PoseWithCovarianceStamped):
        position = pose_msg.pose.pose.position
        orientation = pose_msg.pose.pose.orientation
    else:
        raise ValueError("Unsupported pose type")
    
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    rot = tf.transformations.quaternion_matrix(q)
    trans = tf.transformations.translation_matrix([position.x, position.y, position.z])
    mat = np.dot(trans, rot)
    
    mat = fix_rotation_matrix(mat)
    return mat


def mat_to_pose(mat):
    """矩阵转位姿（反向修正坐标系）"""
    mat = np.copy(mat)
    
    trans = tf.transformations.translation_from_matrix(mat)
    point = Point(x=trans[0], y=trans[1], z=trans[2])
    
    rot = mat[:3, :3]
    # 反向修正旋转
    rot_correction = tf.transformations.euler_matrix(0, 0, -INIT_POSE_ROT_CORRECTION)
    rot = np.dot(rot, rot_correction[:3, :3])
    
    # 正交化修正
    u, s, vh = np.linalg.svd(rot)
    rot = np.dot(u, vh)
    
    rot_mat_full = np.eye(4)
    rot_mat_full[:3, :3] = rot
    q = tf.transformations.quaternion_from_matrix(rot_mat_full)
    quat = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    
    return point, quat


def publish_tf(mat, stamp):
    if not PUBLISH_TF or tf_broadcaster is None:
        return
    
    t = tf2_ros.TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = FRAME_ID_MAP
    t.child_frame_id = FRAME_ID_BASE
    
    point, quat = mat_to_pose(mat)
    t.transform.translation.x = point.x
    t.transform.translation.y = point.y
    t.transform.translation.z = point.z
    t.transform.rotation.x = quat.x
    t.transform.rotation.y = quat.y
    t.transform.rotation.z = quat.z
    t.transform.rotation.w = quat.w
    
    tf_broadcaster.sendTransform(t)


def msg_to_array_extended(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    
    points = np.column_stack([
        pc_array['x'], pc_array['y'], pc_array['z']
    ]).astype(np.float32).copy()
    
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]
    
    result = {'points': points}
    
    if 'normal_x' in pc_array.dtype.names:
        normals = np.column_stack([
            pc_array['normal_x'], pc_array['normal_y'], pc_array['normal_z']
        ]).astype(np.float32).copy()[mask]
        result['normals'] = normals
    else:
        result['normals'] = np.array([])
    
    if 'gravity_x' in pc_array.dtype.names:
        gravity = np.column_stack([
            pc_array['gravity_x'], pc_array['gravity_y'], pc_array['gravity_z']
        ]).astype(np.float32).copy()[mask]
        result['gravity'] = gravity
    else:
        result['gravity'] = np.array([])
    
    return result


def publish_point_cloud_extended(publisher, header, data):
    if publisher is None or len(data['points']) == 0:
        return
    
    dtype_list = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
    if 'normals' in data and data['normals'] is not None and len(data['normals']) > 0:
        dtype_list.extend([('normal_x', np.float32), ('normal_y', np.float32), ('normal_z', np.float32)])
    if 'gravity' in data and data['gravity'] is not None and len(data['gravity']) > 0:
        dtype_list.extend([('gravity_x', np.float32), ('gravity_y', np.float32), ('gravity_z', np.float32)])
    
    max_points = 500000
    point_count = min(len(data['points']), max_points)
    points = data['points'][:point_count]
    
    pc_data = np.zeros(point_count, dtype=dtype_list)
    pc_data['x'] = points[:, 0]
    pc_data['y'] = points[:, 1]
    pc_data['z'] = points[:, 2]
    pc_data['intensity'] = 1.0
    
    if 'normals' in data and data['normals'] is not None and len(data['normals']) > 0:
        normals = data['normals'][:point_count]
        pc_data['normal_x'] = normals[:, 0]
        pc_data['normal_y'] = normals[:, 1]
        pc_data['normal_z'] = normals[:, 2]
    
    if 'gravity' in data and data['gravity'] is not None and len(data['gravity']) > 0:
        gravity = data['gravity'][:point_count]
        pc_data['gravity_x'] = gravity[:, 0]
        pc_data['gravity_y'] = gravity[:, 1]
        pc_data['gravity_z'] = gravity[:, 2]
    
    msg = ros_numpy.msgify(PointCloud2, pc_data)
    msg.header = header
    msg.header.stamp = rospy.Time.now()
    
    with submap_publish_lock:
        publisher.publish(msg)


def parse_map_async(pc_msg):
    global global_map_np, global_map_normals, global_map_gravity, map_ready
    rospy.loginfo("[DEBUG] Parsing map data (fast async)...")
    parse_start = time.time()
    
    if CACHE_ENABLED and load_map_from_cache():
        map_ready = True
        rospy.loginfo("[DEBUG] Map core data ready in %.3fs (map_ready=True)" % (time.time()-parse_start))
        
        t = threading.Thread(target=build_open3d_pointcloud_async)
        t.daemon = True
        t.start()
        return
    
    map_data = msg_to_array_extended(pc_msg)
    rospy.loginfo("[DEBUG] Map parsed: %d points" % len(map_data['points']))
    
    global_map_np = map_data['points'].copy()
    global_map_normals = map_data.get('normals', np.array([]))
    global_map_gravity = map_data.get('gravity', np.array([]))
    
    map_ready = True
    rospy.loginfo("[DEBUG] Map core data ready in %.3fs (map_ready=True)" % (time.time()-parse_start))
    
    t = threading.Thread(target=build_open3d_pointcloud_async)
    t.daemon = True
    t.start()
    if CACHE_ENABLED:
        t2 = threading.Thread(target=save_map_to_cache)
        t2.daemon = True
        t2.start()


def quick_publish_submap(current_pos, header):
    global global_map_np, global_map_normals, global_map_gravity, index_ready, map_ready
    
    if not map_ready or not index_ready or global_map_np is None:
        rospy.logwarn("[DEBUG] Map/FAISS not ready, skip submap publish")
        return
    
    start_time = time.time()
    try:
        idx = query_faiss_ball_point(current_pos, DISPLAY_RADIUS)
        # 动态调整查询半径（补丁5：进一步放宽，保证探维360有足够配准点）
        if len(idx) < 2000:
            idx = query_faiss_ball_point(current_pos, DISPLAY_RADIUS * 3)
        elif len(idx) > 200000:
            idx = query_faiss_ball_point(current_pos, DISPLAY_RADIUS / 2)
        
        if len(idx) == 0:
            rospy.logwarn("[DEBUG] No points for display")
            return
        
        quick_data = {'points': global_map_np[idx]}
        if global_map_normals is not None and len(global_map_normals) > 0:
            quick_data['normals'] = global_map_normals[idx]
        if global_map_gravity is not None and len(global_map_gravity) > 0:
            quick_data['gravity'] = global_map_gravity[idx]
        
        publish_point_cloud_extended(pub_submap, header, quick_data)
        rospy.loginfo("[DEBUG] Submap published: %d points, time=%.3fs" % (len(idx), time.time()-start_time))
    except Exception as e:
        rospy.logerr("[DEBUG] Quick publish error: %s" % str(e))


def get_registration_submap(current_pos, pose_estimation, cur_odom):
    global global_map_np, global_map_normals, index_ready, map_ready
    
    if not map_ready or not index_ready or global_map_np is None:
        rospy.logwarn("[DEBUG] Map/FAISS not ready, skip registration")
        return o3d.geometry.PointCloud()
    
    # 动态调整配准半径（补丁6：适配探维360，进一步放宽）
    idx = query_faiss_ball_point(current_pos, REGISTRATION_RADIUS)
    if len(idx) < 2000:
        idx = query_faiss_ball_point(current_pos, REGISTRATION_RADIUS * 3)
    elif len(idx) > 200000:
        idx = query_faiss_ball_point(current_pos, REGISTRATION_RADIUS / 2)
    
    if len(idx) == 0:
        rospy.logwarn("[DEBUG] No points for registration")
        return o3d.geometry.PointCloud()
    
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.dot(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)
    
    local_points = global_map_np[idx]
    local_points_h = np.column_stack([local_points, np.ones(len(local_points))])
    points_in_base_link = np.dot(T_base_link_to_map, local_points_h.T).T
    
    # 补丁7：适配探维360 360度激光，移除角度过滤（仅保留距离过滤）
    mask = (
        (points_in_base_link[:, 0] > -(FOV_FAR + FOV_FAR_offset)) &
        (points_in_base_link[:, 0] < (FOV_FAR + FOV_FAR_offset)) &
        (points_in_base_link[:, 1] > -(FOV_FAR + FOV_FAR_offset)) &
        (points_in_base_link[:, 1] < (FOV_FAR + FOV_FAR_offset))
    )
    
    reg_submap = o3d.geometry.PointCloud()
    reg_submap.points = o3d.utility.Vector3dVector(local_points[mask])
    
    if global_map_normals is not None and len(global_map_normals) > 0:
        reg_submap.normals = o3d.utility.Vector3dVector(global_map_normals[idx][mask])
    
    if MAP_VOXEL_SIZE > 0 and len(reg_submap.points) > 10000:
        reg_submap = reg_submap.voxel_down_sample(MAP_VOXEL_SIZE)
    
    rospy.loginfo("[DEBUG] Registration submap: %d points" % len(reg_submap.points))
    return reg_submap


def inverse_se3(trans):
    """计算SE(3)矩阵的逆"""
    trans_inverse = np.eye(4)
    trans_inverse[:3, :3] = trans[:3, :3].T
    trans_inverse[:3, 3] = -np.dot(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def registration_at_scale(pc_scan, pc_map, initial, scale):
    """补丁8：适配探维360的ICP参数（增加迭代次数，放宽距离阈值）"""
    if len(pc_scan.points) == 0 or len(pc_map.points) == 0:
        return np.eye(4), 0.0
    
    rospy.loginfo("[DEBUG] ICP start: scan=%d, map=%d, scale=%.1f" % (len(pc_scan.points), len(pc_map.points), scale))
    
    # 第一处修改：去掉 pipelines.
    estimation_method = o3d.registration.TransformationEstimationPointToPoint()
    
    # 补丁：ICP参数适配探维360（迭代次数从50→100，放宽收敛条件）
    # 第二处修改：去掉 pipelines.
    icp_result = o3d.registration.registration_icp(
        pc_scan, pc_map, scale * 0.5, initial,  # 补丁：距离阈值×2
        estimation_method,
        # 第三处修改：去掉 pipelines.
        o3d.registration.ICPConvergenceCriteria(
            max_iteration=100,  # 补丁：迭代次数增加到100
            relative_fitness=1e-5,  # 补丁：放宽收敛条件
            relative_rmse=1e-5
        )
    )
    
    trans = np.copy(icp_result.transformation)
    trans = fix_rotation_matrix(trans)
    
    rospy.loginfo("[DEBUG] ICP done: fitness=%.4f, rmse=%.4f" % (icp_result.fitness, icp_result.inlier_rmse))
    return trans, icp_result.fitness



def crop_scan_in_FOV(cur_scan, cur_odom):
    """补丁9：适配探维360的扫描裁剪（移除角度过滤，仅保留距离过滤）"""
    global scan_valid
    
    if cur_scan is None or len(cur_scan.points) == 0:
        scan_valid = False
        return o3d.geometry.PointCloud()
    
    # 直接基于激光自身坐标系裁剪（核心修复）
    scan_points = np.asarray(cur_scan.points)
    if len(scan_points) == 0:
        scan_valid = False
        return o3d.geometry.PointCloud()
    
    # 补丁：适配探维360，仅保留距离过滤（去掉角度过滤）
    mask = (
        (scan_points[:, 0] > 0.1) &  # 去掉激光原点附近噪声
        (np.linalg.norm(scan_points, axis=1) < FOV_FAR)  # 仅限制最大距离
    )
    
    cropped_scan = o3d.geometry.PointCloud()
    cropped_points = scan_points[mask]
    cropped_scan.points = o3d.utility.Vector3dVector(cropped_points)
    
    if cur_scan.has_normals() and len(np.asarray(cur_scan.normals)) > 0:
        cropped_scan.normals = o3d.utility.Vector3dVector(np.asarray(cur_scan.normals)[mask])
    
    if SCAN_VOXEL_SIZE > 0 and len(cropped_scan.points) > 1000:
        cropped_scan = cropped_scan.voxel_down_sample(SCAN_VOXEL_SIZE)
    
    scan_valid = len(cropped_scan.points) > 0
    rospy.loginfo("[DEBUG] Cropped scan: %d points, valid=%s" % (len(cropped_scan.points), str(scan_valid)))
    return cropped_scan


def global_localization(pose_estimation):
    """补丁10：修复initialized标记更新逻辑，适配探维360的低fitness"""
    global global_map, cur_scan, cur_odom, T_map_to_odom, initialized, index_ready, map_ready, scan_valid
    
    with global_lock:
        if not map_ready or not index_ready or global_map_np is None:
            rospy.logwarn("[DEBUG] Map/FAISS not ready, skip localization")
            return False
        
        if not scan_valid or cur_scan is None or len(cur_scan.points) == 0:
            rospy.logwarn("[DEBUG] Invalid scan data, skip localization")
            return False
        
        if cur_odom is None:
            rospy.logwarn("[DEBUG] Missing odom")
            return False
    
    rospy.loginfo('[DEBUG] ======== Global localization START ========')
    total_start = time.time()
    
    # 计算当前位置（基于新的初始位姿）
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.dot(pose_estimation, T_odom_to_base_link)
    T_map_to_base_link = fix_rotation_matrix(T_map_to_base_link)
    current_pos = T_map_to_base_link[:3, 3]
    
    # 构建header
    header = cur_odom.header
    header.frame_id = FRAME_ID_MAP
    header.stamp = rospy.Time.now()
    
    # 异步发布新的红色点云（基于新位置查询）
    t = threading.Thread(target=quick_publish_submap, args=(current_pos, header))
    t.daemon = True
    t.start()
    
    # 裁剪扫描
    scan_in_FOV = crop_scan_in_FOV(cur_scan, cur_odom)
    if not scan_valid:
        rospy.logwarn("[DEBUG] Empty scan after cropping")
        return False
    
    # 获取新的配准子图
    reg_submap = get_registration_submap(current_pos, pose_estimation, cur_odom)
    if len(reg_submap.points) == 0:
        rospy.logwarn("[DEBUG] Empty registration submap")
        return False
    
    # ICP配准（补丁11：适配探维360的scale参数）
    trans_coarse, fitness_coarse = registration_at_scale(scan_in_FOV, reg_submap, pose_estimation, 3.0)  # 补丁：scale从2→3
    trans_fine, fitness_fine = registration_at_scale(scan_in_FOV, reg_submap, trans_coarse, 1.0)        # 补丁：scale从0.3→1.0
    
    # 判断是否更新（补丁12：大幅放宽探维360的更新条件）
    update_transform = False
    with global_lock:
        t_err = np.linalg.norm(T_map_to_odom[:3, 3] - trans_fine[:3, 3])
        rot_err = np.linalg.norm(T_map_to_odom[:3, :3] - trans_fine[:3, :3])
        
        if not initialized:
            # 初始定位/重新初始化：补丁：超宽松条件适配探维360
            if fitness_fine > LOCALIZATION_TH_init and t_err < 50.0 and rot_err < 5.0:
                update_transform = True
                rospy.loginfo("[DEBUG] Initial localization success: fitness=%.4f, t_err=%.2f, rot_err=%.2f" % (fitness_fine, t_err, rot_err))
        else:
            # 后续微调：补丁：放宽条件适配探维360
            if fitness_fine > LOCALIZATION_TH_max and t_err < 0.1 and rot_err < 0.05:
                update_transform = True

            else:
                rospy.logwarn("[DEBUG] Skip fine update: t_err=%.2f, rot_err=%.2f, fitness=%.4f" % (t_err, rot_err, fitness_fine))
    
    # 更新位姿（核心修复：设置initialized=True）
    if update_transform:
        with global_lock:
            T_map_to_odom = trans_fine
            initialized = True  # 关键：标记为已初始化
        
        map_to_odom = Odometry()
        point, quat = mat_to_pose(T_map_to_odom)
        map_to_odom.pose.pose = Pose(point, quat)
        map_to_odom.header.stamp = header.stamp
        map_to_odom.header.frame_id = FRAME_ID_MAP
        map_to_odom.child_frame_id = FRAME_ID_BASE
        if pub_map_to_odom:
            pub_map_to_odom.publish(map_to_odom)
        
        publish_tf(T_map_to_odom, header.stamp)
    
    rospy.loginfo('[DEBUG] Localization total time: %.3fs, fitness=%.4f' % (time.time()-total_start, fitness_fine))
    rospy.loginfo('[DEBUG] ======== Global localization END ========')
    return update_transform


def initial_pose_callback(msg):
    """监听/initialpose话题，触发重新初始化"""
    global new_initial_pose
    with global_lock:
        new_initial_pose = msg
    rospy.loginfo("[DEBUG] Received new initial pose (trigger re-initialization)")


def initialize_global_map(pc_msg):
    global map_ready
    
    if CACHE_ENABLED:
        create_cache_dir()
    
    parse_thread = threading.Thread(target=parse_map_async, args=(pc_msg,))
    parse_thread.daemon = True
    parse_thread.start()
    
    index_thread = threading.Thread(target=build_faiss_index_async)
    index_thread.daemon = True
    index_thread.start()
    
    rospy.loginfo('[DEBUG] Map init: fast parallel mode (core data first)')


def cb_save_cur_odom(odom_msg):
    global cur_odom
    with global_lock:
        cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan, scan_valid
    
    try:
        if pub_pc_in_map:
            pc_msg.header.frame_id = FRAME_ID_BASE
            pc_msg.header.stamp = rospy.Time.now()
            pub_pc_in_map.publish(pc_msg)
        
        if len(pc_msg.fields) >= 8:
            pc_msg.fields[3].offset = 12
        
        scan_data = msg_to_array_extended(pc_msg)
        if len(scan_data['points']) == 0:
            with global_lock:
                scan_valid = False
            rospy.logwarn("[DEBUG] Empty scan data received")
            return
        
        cur_scan = o3d.geometry.PointCloud()
        cur_scan.points = o3d.utility.Vector3dVector(scan_data['points'])
        if scan_data['normals'] is not None and len(scan_data['normals']) > 0:
            cur_scan.normals = o3d.utility.Vector3dVector(scan_data['normals'])
        
        with global_lock:
            scan_valid = True
    except Exception as e:
        rospy.logerr("[DEBUG] cb_save_cur_scan error: %s" % str(e))
        with global_lock:
            scan_valid = False
            cur_scan = None


def do_localization_attempt(trans_map_to_odom):
    """单次定位尝试（纯时间判断实现超时，无signal依赖）"""
    result = [False]
    exception = [None]
    
    def localization_worker():
        try:
            result[0] = global_localization(trans_map_to_odom)
        except Exception as e:
            exception[0] = e
    
    # 启动子线程执行定位
    worker_thread = threading.Thread(target=localization_worker)
    worker_thread.start()
    # 等待指定时间或线程完成
    worker_thread.join(timeout=ICP_TIMEOUT)
    
    # 如果线程还在运行，视为超时
    if worker_thread.is_alive():
        rospy.logwarn("[DEBUG] Localization attempt timeout (> %ds)" % ICP_TIMEOUT)
        return False
    # 如果有异常，返回False
    if exception[0] is not None:
        rospy.logerr("[DEBUG] Localization attempt error: %s" % str(exception[0]))
        return False
    # 返回结果
    return result[0]


def reinitialization_thread():
    """修复：重新初始化线程（无timeout_decorator依赖）"""
    global initialized, new_initial_pose, T_map_to_odom
    rospy.loginfo("[DEBUG] Re-initialization thread started (listen to /initialpose)")
    
    while not rospy.is_shutdown():
        with global_lock:
            trigger_reinit = new_initial_pose is not None
        
        if trigger_reinit:
            with global_lock:
                current_initial_pose = copy.deepcopy(new_initial_pose)
                initialized = False
                T_map_to_odom = np.eye(4)
            
            rospy.loginfo("[DEBUG] Start re-initialization (attempts: %d)" % MAX_ATTEMPTS)
            attempt_count = 0
            success = False
            
            # 带间隔的重试逻辑（修复：避免阻塞）
            while attempt_count < MAX_ATTEMPTS and not success and not rospy.is_shutdown():
                attempt_count += 1
                rospy.logwarn("[DEBUG] Re-initialization attempt %d/%d" % (attempt_count, MAX_ATTEMPTS))
                
                try:
                    # 核心修复：正确计算初始位姿矩阵
                    initial_pose = initial_pose_to_mat(current_initial_pose)
                    with global_lock:
                        if cur_odom is not None:
                            T_odom_to_base = pose_to_mat(cur_odom)
                            T_base_to_odom = inverse_se3(T_odom_to_base)
                            # 正确矩阵顺序：T_map_to_odom = T_map_to_base * T_base_to_odom
                            trans_map_to_odom = np.dot(initial_pose, T_base_to_odom)
                        else:
                            trans_map_to_odom = initial_pose
                    
                    # 单次ICP尝试（无signal依赖的超时控制）
                    success = do_localization_attempt(trans_map_to_odom)
                except Exception as e:
                    rospy.logerr("[DEBUG] Re-initialization error: %s" % str(e))
                    success = False
                
                time.sleep(0.2)  # 重试间隔，避免资源耗尽
            
            if success:
                rospy.loginfo("[DEBUG] Re-initialization success! Red pointcloud updated.")
            else:
                rospy.logerr("[DEBUG] Re-initialization failed after %d attempts" % MAX_ATTEMPTS)
            
            with global_lock:
                new_initial_pose = None  # 重置
        
        time.sleep(0.1)


def thread_localization():
    """常规定位线程：仅处理后续微调"""
    rate = rospy.Rate(FREQ_LOCALIZATION)
    while not rospy.is_shutdown():
        try:
            with global_lock:
                is_initialized = initialized
                current_T_map_to_odom = copy.deepcopy(T_map_to_odom)
            
            if is_initialized and cur_odom is not None:
                global_localization(current_T_map_to_odom)
            rate.sleep()
        except Exception as e:
            rospy.logerr("[DEBUG] Localization thread error: %s" % str(e))
            rate.sleep()


def main():
    global pub_pc_in_map, pub_submap, pub_map_to_odom, tf_broadcaster
    
    rospy.init_node('fast_lio_localization', anonymous=True)
    rospy.loginfo('Localization Node (Fixed Version: No Timeout Decorator + TW360 Patch + Python2)')
    
    # 初始化TF
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # 创建发布者
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1, latch=True)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1, latch=True)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1, latch=True)
    
    # 订阅关键话题
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, initial_pose_callback, queue_size=1)
    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    
    # 等待全局地图
    rospy.logwarn('Waiting for global map...')
    try:
        map_msg = rospy.wait_for_message('/map_env', PointCloud2, timeout=60)
        initialize_global_map(map_msg)
    except rospy.ROSException:
        rospy.logerr('Timeout waiting for global map!')
        return
    
    # 等待里程计和扫描
    odom_wait_start = time.time()
    while True:
        with global_lock:
            odom_ready = cur_odom is not None
        if odom_ready or rospy.is_shutdown():
            break
        rospy.loginfo_throttle(5.0, '[DEBUG] Waiting for odometry...')
        rospy.sleep(0.1)
    rospy.loginfo("[DEBUG] Odometry received after %.2fs" % (time.time()-odom_wait_start))
    
    scan_wait_start = time.time()
    while True:
        with global_lock:
            scan_ready = cur_scan is not None
        if scan_ready or rospy.is_shutdown():
            break
        rospy.loginfo_throttle(5.0, '[DEBUG] Waiting for scan...')
        rospy.sleep(0.1)
    rospy.loginfo("[DEBUG] Scan received after %.2fs" % (time.time()-scan_wait_start))
    
    # 等待地图和FAISS索引就绪
    rospy.logwarn('[DEBUG] Waiting for map core data + FAISS index (3s target)...')
    wait_start = time.time()
    while True:
        with global_lock:
            map_index_ready = map_ready and index_ready
        if map_index_ready or time.time() - wait_start > POLL_TIMEOUT or rospy.is_shutdown():
            break
        elapsed = time.time() - wait_start
        if int(elapsed) % 1 == 0:
            rospy.loginfo("[DEBUG] Waiting... (elapsed: %.1fs, map=%s, index=%s)" % (elapsed, str(map_ready), str(index_ready)))
        time.sleep(0.2)
    
    if not (map_ready and index_ready):
        rospy.logerr("[DEBUG] Timeout waiting for map+FAISS (>%ds)" % POLL_TIMEOUT)
        return
    rospy.loginfo("[DEBUG] Map+FAISS ready in %.2fs" % (time.time()-wait_start))
    
    # 启动核心线程
    t1 = threading.Thread(target=reinitialization_thread)
    t1.daemon = True
    t1.start()
    t2 = threading.Thread(target=thread_localization)
    t2.daemon = True
    t2.start()
    
    # 等待第一次初始位姿
    rospy.logwarn("[DEBUG] Waiting for first initial pose (attempts: %d)" % MAX_ATTEMPTS)
    attempt_count = 0
    success = False
    while attempt_count < MAX_ATTEMPTS and not success and not rospy.is_shutdown():
        attempt_count += 1
        try:
            first_pose = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped, timeout=POSE_WAIT_TIMEOUT)
            with global_lock:
                new_initial_pose = first_pose
            # 等待重新初始化结果
            wait_count = 0
            while wait_count < 50 and not rospy.is_shutdown():
                with global_lock:
                    success = initialized
                if success:
                    break
                time.sleep(0.1)
                wait_count += 1
        except rospy.ROSException:
            rospy.logwarn("[DEBUG] First initial pose timeout (attempt %d/%d)" % (attempt_count, MAX_ATTEMPTS))
            continue
    
    if success:
        rospy.loginfo('\n[DEBUG] First localization success! Ready for re-initialization.')
    else:
        rospy.logerr("[DEBUG] First localization failed after %d attempts" % MAX_ATTEMPTS)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[DEBUG] Unhandled exception: %s" % str(e))
        raise
