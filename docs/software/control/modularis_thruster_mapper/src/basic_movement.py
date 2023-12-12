#!/usr/bin/env python3
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from modularis_thruster_mapper.msg import PoseTwistStamped

        
def move_forward(odom_msg, pose_twist_msg, forward_movement):
   """Edits trajectory with desired forward_movement given current position from odometry message
   Inputs:   
      odom_msg: Odometry message type for current position of AUV
      pose_twist_msg: PoseTwistStamped message type for holding desired position of AUV
      forward_movement: Integer value of forward movement
   Output: 
      pose_twist_msg: PoseTwistStamped message type with x parameter modified"""
   pose_twist_msg.posetwist.pose.position.x = odom_msg.pose.pose.position.x + forward_movement
   pose_twist_msg.posetwist.pose.position.y = odom_msg.pose.pose.position.y
   pose_twist_msg.posetwist.pose.position.z = odom_msg.pose.pose.position.z
   return pose_twist_msg 

def submerge(odom_msg, pose_twist_msg, down_movement):
   """Edits trajectory with desired down_movement given current position from odometry message
   Inputs:   
      odom_msg: Odometry message type for current position of AUV
      pose_twist_msg: PoseTwistStamped message type for holding desired position of AUV
      forward_movement: Integer value of down movement
   Output: 
      pose_twist_msg: PoseTwistStamped message type with z parameter modified"""
   pose_twist_msg.posetwist.pose.position.x = odom_msg.pose.pose.position.x
   pose_twist_msg.posetwist.pose.position.y = odom_msg.pose.pose.position.y
   pose_twist_msg.posetwist.pose.position.z = odom_msg.pose.pose.position.z - down_movement
   return pose_twist_msg 

def surface(odom_msg, pose_twist_msg, up_movement):
   """Edits trajectory with desired up_movement given current position from odometry message
   Inputs:   
      odom_msg: Odometry message type for current position of AUV
      pose_twist_msg: PoseTwistStamped message type for holding desired position of AUV
      forward_movement: Integer value of up movement
   Output: 
      pose_twist_msg: PoseTwistStamped message type with z parameter modified"""
   pose_twist_msg.posetwist.pose.position.x = odom_msg.pose.pose.position.x 
   pose_twist_msg.posetwist.pose.position.y = odom_msg.pose.pose.position.y 
   pose_twist_msg.posetwist.pose.position.z = odom_msg.pose.pose.position.z + up_movement
   return pose_twist_msg 
