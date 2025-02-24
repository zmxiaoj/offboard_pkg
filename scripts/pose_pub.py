#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
import tf.transformations as tf

class VinsPoseMonitor:
    def __init__(self):
        rospy.init_node('vins_pose_monitor')
        
        # 从参数服务器加载外参
        self.load_extrinsic_params()
        
        # 只保留订阅器
        self.odom_sub = rospy.Subscriber('/vins_estimator/imu_propagate', Odometry, self.odom_callback)
        
        rospy.loginfo("VINS pose monitor node initialized")
        
        # 添加打印控制
        self.print_interval = rospy.Duration(0.5)  # 每0.5秒打印一次
        self.last_print_time = rospy.Time.now()

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
        
        # # 应用坐标变换
        # T_w2b = np.eye(4)
        # T_w2b[:3, :3] = self.R_cam2map @ T_w2c[:3, :3] 
        # T_w2b[:3, 3] = T_w2c[:3, 3]
        
        # TODO
        # T_w2b = self.T_cam2body @ T_w2b
        T_w2c = T_w2c @ self.T_cam2body
        
        # 提取变换后的位置和姿态
        position = T_w2c[:3, 3]
        quaternion = tf.quaternion_from_matrix(T_w2c)
        
        return position, quaternion

    def odom_callback(self, msg):
        """处理VINS里程计消息"""
        current_time = rospy.Time.now()
        if current_time - self.last_print_time > self.print_interval:
            # 执行坐标变换
            pos, quat = self.transform_pose(msg.pose.pose.position, 
                                          msg.pose.pose.orientation)
            
            # 创建简单的位姿数据结构用于打印
            source_pose = msg.pose.pose
            transformed_pose_position = {'x': pos[0], 'y': pos[1], 'z': pos[2]}
            transformed_pose_orientation = {'w': quat[0], 'x': quat[1], 'y': quat[2], 'z': quat[3]}
            
            # 打印信息
            self.print_pose_info(source_pose, transformed_pose_position, transformed_pose_orientation)
            self.last_print_time = current_time

    def print_pose_info(self, source_pose, trans_pos, trans_quat):
        """格式化打印位姿信息"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("VINS Position and Pose:")
        
        # 原始VINS位姿
        rospy.loginfo("\nSource VINS Pose:")
        rospy.loginfo("Position [x y z]: [%.3f, %.3f, %.3f]" % 
                     (source_pose.position.x, 
                      source_pose.position.y, 
                      source_pose.position.z))
        rospy.loginfo("JPL Quaternion [w x y z]: [%.4f, %.4f, %.4f, %.4f]" % 
                     (source_pose.orientation.w,
                      source_pose.orientation.x,
                      source_pose.orientation.y,
                      source_pose.orientation.z))

        # 变换后的位姿
        rospy.loginfo("\nTransformed Body Pose:")
        rospy.loginfo("Position [x y z]: [%.3f, %.3f, %.3f]" % 
                     (trans_pos['x'], trans_pos['y'], trans_pos['z']))
        rospy.loginfo("JPL Quaternion [w x y z]: [%.4f, %.4f, %.4f, %.4f]" % 
                     (trans_quat['w'], trans_quat['x'], 
                      trans_quat['y'], trans_quat['z']))
        rospy.loginfo("=" * 50 + "\n")

if __name__ == '__main__':
    try:
        node = VinsPoseMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
