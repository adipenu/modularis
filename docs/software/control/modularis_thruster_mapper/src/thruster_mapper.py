#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32, Int32
import numpy as np
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs
from geometry_msgs.msg import WrenchStamped

class thruster_mapper(Node):
    def __init__(self):
        super().__init__('thruster_mapper')
        
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/wrench',
            self.wrench_cb,
            3)
        
        self.effort_pub = self.create_publisher(
            Float32,
            '/duty_cycle',
            3)
            
        self.thrust_num_pub = self.create_publisher(
            Int32,
            '/thrust_num',
            3)
        
        self.T_thruster1 = self.transform_matrix(np.array([[0.5, 0.5, -0.25]]).T, np.deg2rad(-270), "z") 
        self.T_thruster2 = self.transform_matrix(np.array([[0.5, -0.5, -0.25]]).T, np.deg2rad(-120), "z")
        self.T_thruster3 = self.transform_matrix(np.array([[-0.5, 0.5, -0.25]]).T, np.deg2rad(270), "z")
        self.T_thruster4 = self.transform_matrix(np.array([[-0.5, -0.5, -0.25]]).T, np.deg2rad(120), "z")
        self.T_thruster5 = self.transform_matrix(np.array([[-0.25, 0, -.3]]).T, np.deg2rad(-90), "x")
        self.T_thruster6 = self.transform_matrix(np.array([[0.25, 0, -.3]]).T, np.deg2rad(-90), "x")
        self.desired_wrench = np.array([[0], [10], [10], [0], [0], [0]]) #example wrench 
        #Combine the transformation matrices into thruster_locations
        self.thruster_locations = np.hstack((self.T_thruster1, self.T_thruster2, self.T_thruster3, self.T_thruster4, self.T_thruster5, self.T_thruster6))
        self.pub_efforts()
        

    def pub_efforts(self):
        """Calls thruster_map function and transforms returned thruster efforts to duty_cycle for PWM driver"""
        duty_msg = Float32()
        thrust_num_msg = Int32()
        while True:
           thruster_efforts = self.thruster_map(self.desired_wrench, self.thruster_locations)           
           for i in range(0, 6):
              if abs(thruster_efforts[i]) > 1e+15: #Filtering out very large values
                 thruster_efforts[i] = 0
           max_thrust = np.max(abs(thruster_efforts))                
           if (max_thrust != 0):
              thruster_efforts = thruster_efforts / max_thrust #Normalize to 0-1.0 range based on maximum thruster efforts found
           for i in range(0, 6):
              """
              if abs(thruster_efforts[i]) < 0.0001: #If very low number then make 0
                 thruster_efforts[i] = 0.0 
              """
              thruster_efforts[i] = thruster_efforts[i]*0.0775 + 0.2925 #Max of 0.37 for 1900us pulse, min of 0.215 duty cycle for 1100us 
            
              thrust_num_msg.data = i + 1 
              duty_msg.data = float(thruster_efforts[i]) #Get rid of +i, temp for testing thrust_num sent
              self.thrust_num_pub.publish(thrust_num_msg)
              self.effort_pub.publish(duty_msg)                          
           rclpy.spin_once(self, timeout_sec=1.0)
        	
    def transform_matrix(self, point, angle, rotation_axis):
        """Calculates the transformation matrix given a point, angle, and rotation axis.
        Inputs: point: 3x1 np.NDArray of the point to translate to
                angle: float of the angle to rotate by
                rotation_axis: 3x1 np.NDArray of the axis to rotate about
        Output: T: 4x4 np.NDArray of the transformation matrix"""
        # Calculating the rotation matrix
        if rotation_axis == "x":
            R = np.array(
                [
                    [1, 0, 0],
                    [0, np.cos(angle), -np.sin(angle)],
                    [0, np.sin(angle), np.cos(angle)],
                ]
            )
        elif rotation_axis == "y":
            R = np.array(
                [
                    [np.cos(angle), 0, np.sin(angle)],
                    [0, 1, 0],
                    [-np.sin(angle), 0, np.cos(angle)],
                ]
            )
        elif rotation_axis == "z":
            R = np.array(
                [
                    [np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0, 0, 1],
                ]
            )
        # Calculating the translation matrix
        P = point
        T = np.vstack((np.hstack((R, P)), np.array([0, 0, 0, 1])))
        return T
        
    def deg2rad(self, x): return x * np.pi / 180

    def thruster_map(
        self, desired_wrench, thruster_locations, S=np.array([[0, 1, 0, 0, 0, 0]]).T
    ):
        """Calculates the thruster forces and torques required to achieve the desired wrench.
        Inputs: desired_wrench: 6x1 np.NDArray of the desired wrench
                thruster_locations: 4x(N*4) np.NDArray of the thruster locations in the body frame, where N is
                                    the number of thrusters, and the thruster force acts along the y of the thruster frame.
                S: 6x1 np.NDArray of the thruster directions in the body frame. Defaults to the positive y-axis.
        Output: thruster_efforts: 1xN np.NDArray of the thruster efforts"""
        # Applying the transform to the thruster locations
        S_matrix = np.zeros((6, 0))
        for i in range(int(thruster_locations.shape[1] / 4)):
            T = thruster_locations[:, 4 * i: 4 * i + 4]
            R = T[:3, :3]
            P = np.array([T[:3, 3]]).T
            # Calculating the thruster in the body frame
            S_body = R @ S[:3, :]
            S_OLbody = R @ S[3:7] + np.cross(P, S_body, axis=0)
            Thruster_S = np.vstack((S_body, S_OLbody))
            S_matrix = np.hstack((S_matrix, Thruster_S))
        thruster_efforts = np.linalg.inv(S_matrix) @ desired_wrench
        return thruster_efforts
    
    def wrench_cb(self, msg: WrenchStamped) -> None:
        wrench_data = msg.wrench

    #Extract force and torque components from the received Wrench message
        force = wrench_data.force
        torque = wrench_data.torque
        self.desired_wrench = np.array([
        [force.x],
        [force.y],
        [force.z],
        [torque.x],
        [torque.y],
        [torque.z]])
    

def main(args=None):
    rclpy.init(args=args)
    thrust_map = thruster_mapper()
    rclpy.spin(thrust_map)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
