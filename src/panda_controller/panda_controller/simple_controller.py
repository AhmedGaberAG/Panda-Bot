#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import numpy as np
import math 

class SimpleController(Node):
    def __init__ (self):
        super().__init__("simple_controller")
        # Robot parameters
        self.declare_parameter("wheel_radius", 0.065)
        self.declare_parameter("wheel_separation", 0.37)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.get_logger().info(f"Wheel radius: {self.wheel_radius_:.3f} m")
        self.get_logger().info( f"Wheel separation: {self.wheel_separation_:.3f} m")

        # Previous wheel positions
        # Initialized from the first JointState message
        self.right_wheel_prev_pos_ = 0.0
        self.left_wheel_prev_pos_ = 0.0
        self.prev_time_ = None

        # Robot pose from odometry
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(Twist, "panda_controller/cmd_vel_unstamped", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "panda_controller/odom", 10)

        # Differential-drive speed conversion matrix: r = wheel_radius, L = wheel_separation
        # [v]   = [ r/2   r/2 ] · [φ_R]  
        # [ω]     [ r/L  -r/L ]   [φ_L]
        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2.0, self.wheel_radius_ / 2.0],
            [self.wheel_radius_ / self.wheel_separation_, -self.wheel_radius_ / self.wheel_separation_]
        ])

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.get_logger().info(f"The conversion matrix is {self.speed_conversion_}")

    def velCallback(self, msg):
        # Get the robot velocity vector:
        # [v] = [linear velocity (v)]
        # [ω]   [angular velocity (ω)]
        robot_speed = np.array([[msg.linear.x],
                               [msg.angular.z]])
        # Compute wheel angular velocities:
        # [φ_R] = (conversion matrix)^(-1) · [v]
        # [φ_L]                              [ω]      
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [
            wheel_speed[0, 0],  # speed : right_wheel_joint (φ_R)
            wheel_speed[1, 0]   # speed : left_wheel_joint  (φ_L)
        ]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self , msg):
        # Get wheel joint indices
        right_index = msg.name.index("wheel_right_joint")
        left_index = msg.name.index("wheel_left_joint")

        # Current joint-state timestamp
        current_time = Time.from_msg(msg.header.stamp)

        # Initialize previous values from the first message
        if self.prev_time_ is None:
            self.right_wheel_prev_pos_ = msg.position[right_index]
            self.left_wheel_prev_pos_ = msg.position[left_index]
            self.prev_time_ = current_time
            return

        # Calculate wheel position changes
        dp_right = msg.position[right_index] - self.right_wheel_prev_pos_
        dp_left = msg.position[left_index] - self.left_wheel_prev_pos_

        # Calculate elapsed time
        dt = current_time - self.prev_time_
        dt_seconds = dt.nanoseconds / S_TO_NS

        # Ignore invalid time intervals
        if dt_seconds <= 0.0:
            return

        # Update previous values
        self.right_wheel_prev_pos_ = msg.position[right_index]
        self.left_wheel_prev_pos_ = msg.position[left_index]
        self.prev_time_ = current_time

        # Wheel angular velocities
        fi_right = dp_right / dt_seconds # φ_R = Δθ_R / Δt
        fi_left = dp_left / dt_seconds   # φ_L = Δθ_L / Δt

        # Differential-drive forward kinematics
        # [v]   = [ r/2   r/2 ] · [φ_R]
        # [ω]     [ r/L  -r/L ]   [φ_L]
        # ---
        # v = (r/2) · (φ_R + φ_L)
        # ω = (r/L) · (φ_R - φ_L)
        linear_velocity = (fi_right + fi_left) * (self.wheel_radius_ / 2.0)
        angular_velocity = (fi_right - fi_left) * (self.wheel_radius_ / self.wheel_separation_)

        # Differential-drive odometry
        # Wheel angular displacement:
        # Δθ_R = φ_R · Δt
        # Δθ_L = φ_L · Δt
        # ---
        # Robot linear displacement:
        # Δs = (r/2) · (Δθ_R + Δθ_L)
        # ---
        # Robot angular displacement:
        # Δθ = (r/L) · (Δθ_R - Δθ_L)
        d_s = (dp_right + dp_left) * (self.wheel_radius_ / 2.0)
        d_theta = (dp_right - dp_left) * (self.wheel_radius_ / self.wheel_separation_)
        
        # Update robot orientation
        # θ_new = θ_old + Δθ
        self.theta_ += d_theta

        # Update robot position using the midpoint orientation
        # θ_mid = θ_new - Δθ/2
        # Δx = Δs · cos(θ_mid)
        # Δy = Δs · sin(θ_mid)
        self.x_ += d_s * math.cos(self.theta_ - d_theta / 2.0)
        self.y_ += d_s * math.sin(self.theta_ - d_theta / 2.0)

        # Convert Euler yaw angle to quaternion
        # ROS 2 uses quaternions for orientation
        q = quaternion_from_euler(0, 0, self.theta_)
        
        # Update odometry message
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.position.z = 0.0
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = msg.header.stamp
        
        # Robot velocity
        self.odom_msg_.twist.twist.linear.x = linear_velocity
        self.odom_msg_.twist.twist.angular.z = angular_velocity

        # Update odom -> base_footprint transform
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.translation.z = 0.0
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = msg.header.stamp

        # Publish odometry
        self.odom_pub_.publish(self.odom_msg_)

        # Broadcast odom -> base_footprint TF
        self.br_.sendTransform(self.transform_stamped_)

def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()