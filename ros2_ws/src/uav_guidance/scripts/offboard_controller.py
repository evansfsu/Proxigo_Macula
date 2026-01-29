#!/usr/bin/env python3
# offboard_controller.py
import rclpy
from rclpy.node import Node
import time

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleCommand  # If not present, use px4_msgs.msg.VehicleCommand

# Rate and basic behavior
PUB_RATE_HZ = 20.0
ARM_RETRY_SECS = 1.0
OFFBOARD_TIMEOUT = 1.0  # seconds before re-sending mode/arm

class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')
        self.get_logger().info("Starting OffboardController")
        # Publishers
        self.pub_offboard = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', 10)
        self.pub_setpoint = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(1.0 / PUB_RATE_HZ, self.timer_cb)
        self.last_mode_sent = 0.0
        self.last_arm_sent = 0.0
        self.seq = 0

    def timer_cb(self):
        now = self.get_clock().now().seconds_nanoseconds()[0] + \
              self.get_clock().now().seconds_nanoseconds()[1] / 1e9

        # OffboardControlMode
        if now - self.last_mode_sent > OFFBOARD_TIMEOUT:
            moc = OffboardControlMode()
            # Enable position, velocity setpoints
            moc.position = True
            moc.velocity = True
            moc.acceleration = False
            moc.attitude = False
            moc.body_rate = False
            self.pub_offboard.publish(moc)
            self.last_mode_sent = now
            self.get_logger().debug("Published OffboardControlMode")

        # TrajectorySetpoint (example: hold local origin)
        setpt = TrajectorySetpoint()
        setpt.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        setpt.position = [0.0, 0.0, -10.0]
        setpt.velocity = [0.0, 0.0, 0.0]
        setpt.acceleration = [0.0, 0.0, 0.0]
        setpt.yaw = 0.0
        setpt.yawspeed = 0.0
       
        self.pub_setpoint.publish(setpt)
        self.get_logger().debug("Published TrajectorySetpoint seq=%d" % self.seq)
        self.seq += 1

        # Arm request (retries until success; safer to send with user confirmation in real flights)
        if now - self.last_arm_sent > ARM_RETRY_SECS:
            cmd = VehicleCommand()
            cmd.timestamp = int(self.get_clock().now().nanoseconds // 1000)
            cmd.param1 = 1.0  # arm
            cmd.param2 = 0.0  # magic? (you can also use command_long style)
            cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            cmd.target_system = 1
            cmd.target_component = 1
            
            cmd.source_system = 1
            cmd.source_component = 1
            cmd.from_external = True
            self.pub_vehicle_cmd.publish(cmd)
            self.last_arm_sent = now
            self.get_logger().info("Sent arm command")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
