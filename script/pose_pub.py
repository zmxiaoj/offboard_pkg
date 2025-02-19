#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf

class VinsPoseTransformer:
    def __init__(self):
        rospy.init_node('vins_pose_transformer')
        
        # 从参数服务器加载外参
        self.load_extrinsic_params()
        
        # 发布器和订阅器
        self.pose_pub = rospy.Publisher('/vins/transformed_pose', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, self.odom_callback)
        
        rospy.loginfo("VINS pose transformer node initialized")

    def load_extrinsic_params(self):
        """加载外参配置"""
        # 从YAML加载的外参矩阵
        translation = rospy.get_param('/camera_extrinsic/matrix/translation', [0.0, 0.0, 0.0])
        rotation = rospy.get_param('/camera_extrinsic/matrix/rotation', 
                                 [1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0])

        # 构建4x4变换矩阵
        self.T_body2cam = np.eye(4)
        self.T_body2cam[:3, :3] = np.array(rotation).reshape(3, 3)
        self.T_body2cam[:3, 3] = translation
        
        # 计算逆变换
        self.T_cam2body = np.linalg.inv(self.T_body2cam)
        
        # VINS坐标系到map坐标系的基础变换
        # 这个变换将VINS的相机坐标系对齐到ENU坐标系
        self.R_cam2map = np.array([
            [ 1.0,  0.0,  0.0],
            [ 0.0,  1.0,  0.0],
            [ 0.0,  0.0,  1.0]
        ])
        
        rospy.loginfo("Loaded extrinsic parameters")
        rospy.loginfo(f"Translation: {translation}")
        rospy.loginfo(f"Rotation:\n{np.array(rotation).reshape(3,3)}")

    def transform_pose(self, pos, quat):
        """执行坐标变换"""
        # 构建VINS位姿矩阵
        T_w2c = np.eye(4)
        T_w2c[:3, :3] = tf.quaternion_matrix([quat.w, quat.x, quat.y, quat.z])[:3, :3]
        T_w2c[:3, 3] = [pos.x, pos.y, pos.z]
        
        # 应用坐标变换
        T_w2b = np.eye(4)
        T_w2b[:3, :3] = self.R_cam2map @ T_w2c[:3, :3] 
        T_w2b[:3, 3] = T_w2c[:3, 3]
        
        T_w2b = T_w2b @ self.T_cam2body
        
        # 提取变换后的位置和姿态
        position = T_w2b[:3, 3]
        rotation = T_w2b[:3, :3]
        quaternion = tf.quaternion_from_matrix(T_w2b)
        
        return position, quaternion

    def odom_callback(self, msg):
        """处理VINS里程计消息"""
        # 执行坐标变换
        pos, quat = self.transform_pose(msg.pose.pose.position, 
                                      msg.pose.pose.orientation)
        
        # 创建并发布PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = "map"  # 使用map作为参考系
        
        # 设置位置
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        
        # 设置姿态四元数
        pose_msg.pose.orientation.w = quat[0]
        pose_msg.pose.orientation.x = quat[1]
        pose_msg.pose.orientation.y = quat[2]
        pose_msg.pose.orientation.z = quat[3]
        
        # 发布转换后的位姿
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        node = VinsPoseTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
