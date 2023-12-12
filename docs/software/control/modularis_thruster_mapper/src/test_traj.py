#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from modularis_thruster_mapper.msg import PoseTwistStamped
from basic_movement import move_forward, submerge, surface
import time

class ConstantPoseTwistPublisher(Node):

    def __init__(self):
        super().__init__('test_traj')
        self.publisher_trajectory = self.create_publisher(PoseTwistStamped, 'trajectory', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish_pose_twist)

    def publish_pose_twist(self):
        pose_twist_msg = PoseTwistStamped()
        odom_msg = Odometry()

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        pose_twist_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'random' #Setting frame_id and child_frame_id to random values for now
        odom_msg.child_frame_id = 'rand_child'
        pose_twist_msg.header.frame_id = 'base_link'

        #Initializing parameters for pose_twist_msg and odom_msg 
        pose_twist_msg.posetwist.pose = Pose() 
        pose_twist_msg.posetwist.pose.orientation.x = 0.0
        pose_twist_msg.posetwist.pose.orientation.y = 0.0
        pose_twist_msg.posetwist.pose.orientation.z = 0.0
        pose_twist_msg.posetwist.pose.orientation.w = 1.0

        pose_twist_msg.posetwist.twist = Twist()
        pose_twist_msg.posetwist.twist.linear.x = 0.0
        pose_twist_msg.posetwist.twist.linear.y = 0.0
        pose_twist_msg.posetwist.twist.linear.z = 0.0
        pose_twist_msg.posetwist.twist.angular.x = 0.0
        pose_twist_msg.posetwist.twist.angular.y = 0.0
        pose_twist_msg.posetwist.twist.angular.z = 0.0

        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0

        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 2.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 2.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        #Testing different movements of thrusters
        surface(odom_msg, pose_twist_msg, 10)
        
        self.publisher_odom.publish(odom_msg)
        self.publisher_trajectory.publish(pose_twist_msg)
        
        time.sleep(10)
        
        submerge(odom_msg, pose_twist_msg, 10)
        
        self.publisher_odom.publish(odom_msg)
        self.publisher_trajectory.publish(pose_twist_msg)
        
        time.sleep(10)
        
        surface(odom_msg, pose_twist_msg, 10)
        
        self.publisher_odom.publish(odom_msg)
        self.publisher_trajectory.publish(pose_twist_msg)
        
        time.sleep(10) 
        
        move_forward(odom_msg, pose_twist_msg, 0)
        self.publisher_odom.publish(odom_msg)
        self.publisher_trajectory.publish(pose_twist_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ConstantPoseTwistPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

