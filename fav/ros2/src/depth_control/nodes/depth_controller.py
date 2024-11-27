#!/usr/bin/env python3
"""
This node is your depth controller with a FIR filter.
"""

import rclpy
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from collections import deque


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.init_params()

        self.current_setpoint = 0.0
        self.current_depth = 0.0
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now()

        # FIR Filter parameters
        self.filter_coeffs = [0.2, 0.2, 0.2, 0.2,
                              0.2]  # Example: Moving average
        self.filter_buffer = deque(maxlen=len(self.filter_coeffs))

        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            callback=self.on_setpoint,
            qos_profile=1,
        )
        self.depth_sub = self.create_subscription(
            msg_type=DepthStamped,
            topic='depth',
            callback=self.on_depth,
            qos_profile=1,
        )

    def init_params(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gains.p', rclpy.Parameter.Type.DOUBLE),
                ('gains.i', rclpy.Parameter.Type.DOUBLE),
                ('gains.d', rclpy.Parameter.Type.DOUBLE),
            ],
        )

        # Initializing gains from parameters
        self.p_gain = self.get_parameter('gains.p').value
        self.i_gain = self.get_parameter('gains.i').value
        self.d_gain = self.get_parameter('gains.d').value

        self.get_logger().info(
            f"Initialized gains: P={self.p_gain}, I={self.i_gain}, D={self.d_gain}"
        )

        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params):
        for param in params:
            if param.name == 'gains.p':
                self.p_gain = param.value
            elif param.name == 'gains.i':
                self.i_gain = param.value
            elif param.name == 'gains.d':
                self.d_gain = param.value
            else:
                continue
            self.get_logger().info(
                f"Parameter updated: {param.name} = {param.value}")
        return SetParametersResult(successful=True)

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        raw_depth = depth_msg.depth

        # Apply FIR filter
        filtered_depth = self.apply_fir_filter(raw_depth)
        self.get_logger().info(
            f"Controller running. Raw depth: {raw_depth}, Filtered depth: {filtered_depth}",
            throttle_duration_sec=1,
        )

        thrust = self.compute_control_output(filtered_depth)
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def apply_fir_filter(self, new_value: float) -> float:
        """
        Apply the FIR filter to the new input value.
        """
        # Add the new value to the buffer
        self.filter_buffer.append(new_value)

        # Compute the weighted sum
        filtered_value = sum(
            coeff * val
            for coeff, val in zip(self.filter_coeffs, self.filter_buffer))

        return filtered_value

    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False
        msg.z = thrust
        msg.header.stamp = timestamp.to_msg()
        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        error = self.current_setpoint - current_depth
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        p_term = self.p_gain * error
        self.integral += error * dt
        i_term = self.i_gain * self.integral

        if dt > 0:
            d_term = self.d_gain * (error - self.last_error) / dt
        else:
            d_term = 0.0

        thrust_z = p_term + i_term + d_term

        self.last_error = error
        self.last_time = current_time

        if self.current_setpoint > -0.1 or self.current_setpoint < -0.8:
            return 0.0
        else:
            return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
