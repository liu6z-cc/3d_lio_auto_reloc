# 创建一个发布假点云的脚本
cat > /tmp/publish_fake_pointcloud.py << 'EOF'
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import struct
import numpy as np

def create_point_cloud():
    """创建一个简单的点云"""
    msg = PointCloud2()
    msg.header.frame_id = "camera_init"
    msg.header.stamp = rospy.Time.now()
    
    # 定义点云字段
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    
    msg.is_bigendian = False
    msg.point_step = 12  # 3 floats * 4 bytes
    msg.is_dense = True
    
    # 创建一些点（简单的网格）
    points = []
    for x in np.linspace(-5, 5, 20):
        for y in np.linspace(-5, 5, 20):
            z = 0.0
            points.append(struct.pack('fff', x, y, z))
    
    msg.data = b''.join(points)
    msg.height = 1
    msg.width = len(points)
    msg.row_step = msg.point_step * msg.width
    
    return msg

def create_odometry():
    """创建里程计消息"""
    msg = Odometry()
    msg.header.frame_id = "camera_init"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.orientation.w = 1.0
    return msg

if __name__ == '__main__':
    rospy.init_node('fake_lidar_publisher')
    
    cloud_pub = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=10)
    odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("Publishing fake point cloud and odometry...")
    
    while not rospy.is_shutdown():
        cloud_pub.publish(create_point_cloud())
        odom_pub.publish(create_odometry())
        rate.sleep()
EOF

chmod +x /tmp/publish_fake_pointcloud.py