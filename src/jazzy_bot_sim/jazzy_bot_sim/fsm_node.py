#!/usr/bin/env python3
"""
Reactive FSM Node for JazzyBot (Gazebo Harmonic compatible)

Behavior:
- Move forward if NO obstacle detected in front (±30°)
- Rotate in place WHILE obstacle is detected in front
- Stop rotating ONLY when front is clear
- Emergency stop overrides all

Author: Gading
Date: 2026
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from enum import Enum
import math


class RobotState(Enum):
    WANDER = 1
    AVOID_OBSTACLE = 2
    EMERGENCY_STOP = 3


class FSMNode(Node):

    def __init__(self):
        super().__init__('fsm_controller')

        # ================= PARAMETERS =================
        self.declare_parameter('wander_linear_velocity', 0.3)
        self.declare_parameter('avoid_angular_velocity', 0.6)
        self.declare_parameter('obstacle_threshold', 0.5)  # ↓ turunkan default ke 0.5
        self.declare_parameter('control_frequency', 20.0)

        self.wander_linear_vel = self.get_parameter('wander_linear_velocity').value
        self.avoid_angular_vel = self.get_parameter('avoid_angular_velocity').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        control_freq = self.get_parameter('control_frequency').value

        # ================= PUBLISHERS =================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ================= SUBSCRIBERS =================
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # ================= STATE =================
        self.current_state = RobotState.WANDER
        self.emergency_stop_active = False
        self.min_obstacle_distance = float('inf')
        self.last_state = None

        # ================= CONTROL LOOP =================
        self.timer = self.create_timer(1.0 / control_freq, self.control_loop)
        self.print_startup_info()

    def scan_callback(self, msg: LaserScan):
        """Check obstacle ONLY in front sector (±30 degrees)"""
        if not msg.ranges:
            self.min_obstacle_distance = float('inf')
            return

        total_ranges = len(msg.ranges)
        center_index = total_ranges // 2
        angle_increment = msg.angle_increment

        front_angle_rad = math.radians(30)
        num_samples = int(front_angle_rad / angle_increment)

        start = max(0, center_index - num_samples)
        end = min(total_ranges, center_index + num_samples)

        valid_ranges = []
        MIN_VALID = 0.1  # Ignore noise/self-detection (0.0 or too close)

        for r in msg.ranges[start:end]:
            if (not math.isinf(r) and not math.isnan(r) and
                r >= MIN_VALID and r <= msg.range_max):
                valid_ranges.append(r)

        self.min_obstacle_distance = min(valid_ranges) if valid_ranges else float('inf')

    def emergency_callback(self, msg: Bool):
        if msg.data and not self.emergency_stop_active:
            self.emergency_stop_active = True
            self.current_state = RobotState.EMERGENCY_STOP
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        elif not msg.data and self.emergency_stop_active:
            self.emergency_stop_active = False
            self.current_state = RobotState.WANDER
            self.get_logger().info('Emergency stop released')

    def control_loop(self):
        # Emergency override
        if self.emergency_stop_active:
            self.current_state = RobotState.EMERGENCY_STOP
        else:
            # Reactive logic: switch based on current sensor reading
            if self.min_obstacle_distance < self.obstacle_threshold:
                self.current_state = RobotState.AVOID_OBSTACLE
            else:
                self.current_state = RobotState.WANDER

        # Execute behavior
        if self.current_state == RobotState.WANDER:
            self.handle_wander()
        elif self.current_state == RobotState.AVOID_OBSTACLE:
            self.handle_avoid()
        elif self.current_state == RobotState.EMERGENCY_STOP:
            self.handle_emergency()

        # Log state change only once
        if self.current_state != self.last_state:
            state_names = {
                RobotState.WANDER: "WANDER (moving forward)",
                RobotState.AVOID_OBSTACLE: "AVOID_OBSTACLE (rotating)",
                RobotState.EMERGENCY_STOP: "EMERGENCY_STOP"
            }
            self.get_logger().info(f"→ State changed to: {state_names[self.current_state]}")
            self.last_state = self.current_state

    def handle_wander(self):
        cmd = Twist()
        cmd.linear.x = self.wander_linear_vel
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def handle_avoid(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.avoid_angular_vel  # Putar terus sampai clear
        self.cmd_vel_pub.publish(cmd)

    def handle_emergency(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def print_startup_info(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('REACTIVE FSM CONTROLLER STARTED')
        self.get_logger().info(f'Linear velocity   : {self.wander_linear_vel} m/s')
        self.get_logger().info(f'Angular velocity  : {self.avoid_angular_vel} rad/s')
        self.get_logger().info(f'Obstacle threshold: {self.obstacle_threshold} m')
        self.get_logger().info('=' * 60)

    def destroy_node(self):
        self.get_logger().info('Shutting down FSM, stopping robot...')
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()