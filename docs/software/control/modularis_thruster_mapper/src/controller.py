#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry

import numpy as np
from threading import Lock

# rest of imports i need to search for and convert
from typing import List

from modularis_thruster_mapper.msg import PoseTwistStamped

from mil_tools import (
    numpy_to_wrench, 
    odometry_to_numpy,
    pose_to_numpy, 
    quat_to_rotvec, 
    quaternion_matrix,
    twist_to_numpy,
)

from tf_transformations import quaternion_inverse, quaternion_multiply

class Sub8AdaptiveController(Node):
    def __init__(self):
        """
        Sub8 adaptive controller receives desired state from a
        trajectory generator, and uses the current state of the
        sub (feeedback) to calculate the control input to achieve
        the desired state.

        This is achieved by using a PID controller and other adaptive
        gains.
        Comment: dist == disturbance, est == estimate

        Attributes:
            kp (np.array): proportional gain
            kd (np.array): derivative gain
            ki (np.array): i gain applied to the integral of the twist error
            kg (np.array): g gain applied to the integral of the pose error
            use_learned (bool): whether to use learned disturbances
            last_config (Config): last config sent to reconfigure
            desired_position (np.array): desired position
            desired_orientation (np.array): desired orientation
            desired_twist_world (np.array): desired twist in world frame
            position (np.array): current position
            orientation (np.array): current orientation
            twist_body (np.array): current twist in body frame
            twist_world (np.array): current twist in world frame
            body_to_world (np.array): body to world transformation matrix
            world_to_body (np.array): world to body transformation matrix
            dist_est_world (np.array): disturbance estimate in world frame
            drag_est_body (np.array): drag estimate in body frame
            learning_distance (float): maximum distance of error to learn
            body_frame (str): body frame id
            global_frame (str): global frame id
        """
        super().__init__('adaptive_controller')

        np.set_printoptions(precision=2)

        # Initialize gains
        self.kp = np.array([500, 500, 500, 1000, 1000, 1000])
        self.kd = np.array([100,100,100,150,150,150])
        self.ki = np.array([5,5,5,5,5,5])
        self.kg = np.array([5,5,5,5,5,5])
        self.learning_distance = (
            1.0  
        )
        self.use_learned = False

        # Initialize state variables
        self.dist_est_world = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
        self.drag_est_body = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
        self.body_frame = None
        self.global_frame = None
        self.last_update = None
        self.desired_position = None
        self.desired_orientation = None
        self.desired_twist_world = None
        self.body_to_world = None
        self.world_to_body = None
        self.position = None
        self.orientation = None
        self.twist_world = None
        self.pose = None
        self.twist_world = None
        self.twist_body = None

        self.lock = Lock()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            3
        )
        self.ref = self.create_subscription(
            PoseTwistStamped,
            '/trajectory',
            self.trajectory_cb,
            3
        )
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/wrench',
            3
        )

    @staticmethod
    def parse_gains(gains) -> List[float]:
        ret = [float(gain.strip()) for gain in gains.split(",")]
        if len(ret) != 6:
            raise ValueError("not 6")
        return ret
    
    def update(self) -> None:
        """Computes the pose and twist errors, updates the regressors (estimates),
        and calculates and publishes the desired wrench to be applied based on the
        PD gains, the estimated drag in the body frame and the estimated disturbances
        in the world frame.
        """
        nowmsg = self.get_clock().now().to_msg() #this could be issue
        now = self.get_clock().now()
        # Send a zeroed wrench if killed, missing odometry, or missing goal
        if (
            self.desired_position is None
            or self.position is None
        ):
            zero_wrench = np.zeros(6, dtype=float)
            zero_pub = self.make_wrench_msg(zero_wrench, frame_id=self.body_frame, stamp=nowmsg)
            self.wrench_pub.publish(
                zero_pub,
            )
            return

        # Calculate error in position, orientation, and twist
        position_error_world = self.desired_position - self.position
        orientation_error_world = quat_to_rotvec(
            quaternion_multiply(
                self.desired_orientation,
                quaternion_inverse(self.orientation),
            ),
        )
        pose_error_world = np.concatenate(
            (position_error_world, orientation_error_world),
        )
        twist_error_world = self.desired_twist_world - self.twist_world
        twist_error_body = self.world_to_body.dot(twist_error_world)

        # Rotate gains into world frame
        kp = self.body_to_world.dot(np.diag(self.kp)).dot(self.world_to_body)
        kd = self.body_to_world.dot(np.diag(self.kd)).dot(self.world_to_body)

        # Calculate part of wrench from feedback
        feedback_proportional_world = kp.dot(pose_error_world)
        feedback_derivative_world = kd.dot(twist_error_world)
        feedback_world = feedback_proportional_world + feedback_derivative_world

        # Calculate part of wrench from learned parameters
        drag_effort_body = np.multiply(self.drag_est_body, self.twist_body)
        drag_effort_world = self.body_to_world.dot(drag_effort_body)
        wrench_adaptation_world = drag_effort_world + self.dist_est_world

        if self.use_learned:
            # If learning is on, total wrench is feedback + learned integrator
            wrench_world = wrench_adaptation_world + feedback_world
        else:
            # if learning is off, wrench is just from feedback
            wrench_world = feedback_world

        # Convert wrench to body frame as per ROS convention
        wrench_body = self.world_to_body.dot(wrench_world)

        # Publish wrench
        body_pub = self.make_wrench_msg(wrench_body, frame_id=self.body_frame, stamp=nowmsg)
        self.wrench_pub.publish(
            body_pub,
        )

        # Update regressors
        dist = np.linalg.norm(pose_error_world)
        if (
            self.last_update is not None
            and dist < self.learning_distance
            and self.use_learned
        ):
            dt = (now - self.last_update).nanoseconds
            self.drag_est_body = (
                self.drag_est_body + np.multiply(self.ki, twist_error_body) * dt
            )
            self.dist_est_world = (
                self.dist_est_world + np.multiply(self.kg, pose_error_world) * dt
            )
            # print 'Drag est', self.drag_est_body
            # print 'Dist est', self.dist_est_world
        self.last_update = now

    @staticmethod
    def make_wrench_msg(
        wrench: np.ndarray,
        frame_id: str = "base_link",
        stamp=None
    ) -> WrenchStamped:
        """Creates a WrenchStamped message from a numpy array."""
        msg = WrenchStamped()
        msg.header.stamp = stamp 
        msg.header.frame_id = frame_id
        wrench_msg = geometry_msgs.Vector3()
        wrench_msg.x = wrench[0]
        wrench_msg.y = wrench[1]
        wrench_msg.z = wrench[2]
        torque_msg = geometry_msgs.Vector3()
        torque_msg.x = wrench[3]
        torque_msg.y = wrench[4]
        torque_msg.z = wrench[5]
        msg.wrench.force.x=wrench_msg.x
        msg.wrench.force.y=wrench_msg.y
        msg.wrench.force.z=wrench_msg.z
        msg.wrench.torque.x=torque_msg.x 
        msg.wrench.torque.y=torque_msg.y 
        msg.wrench.torque.z=torque_msg.z 
        return msg

    @staticmethod
    def make_double_rotation(R: np.ndarray) -> np.ndarray:
        """Creates a double rotation matrix from a rotation matrix. This
        is useful to rotate two vectors in a single operation.
        """
        R2 = np.zeros((6, 6))
        R2[:3, :3] = R2[3:, 3:] = R
        return R2

    def trajectory_cb(self, msg: PoseTwistStamped) -> None:
        """Callback for the trajectory topic to get the desired state."""
        with self.lock:
            self.desired_position, self.desired_orientation = pose_to_numpy(
                msg.posetwist.pose,
            )
            body_to_world = self.make_double_rotation(
                quaternion_matrix(self.desired_orientation),
            )
            self.desired_twist_world = body_to_world.dot(
                np.hstack(twist_to_numpy(msg.posetwist.twist)),
            )

    def odom_cb(self, msg: Odometry) -> None:
        """Callback for the odometry topic to get the current state."""
        with self.lock:
            self.body_frame = msg.child_frame_id
            self.global_frame = msg.header.frame_id
            (
                (self.position, self.orientation),
                (linvel, angvel),
                _,
                _,
            ) = odometry_to_numpy(msg)
            self.body_to_world = self.make_double_rotation(
                quaternion_matrix(self.orientation),
            )
            self.world_to_body = self.body_to_world.T
            self.twist_body = np.concatenate((linvel, angvel))
            self.twist_world = self.body_to_world.dot(self.twist_body)
            self.update()
            
            


def main(args=None):
    rclpy.init(args=args)
    sub8_controller = Sub8AdaptiveController()
    rclpy.spin(sub8_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
