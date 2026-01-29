#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

class OffboardHello(Node):
    """
    Minimal Offboard publisher:
    - streams setpoints
    - arms
    - sets Offboard mode

    Notes:
    - PX4 requires setpoints to be streamed before entering Offboard.
    - Fixed-wing may not "takeoff" like multicopter; in SITL, you can still validate message flow.
    """

    def __init__(self):
        super().__init__("offboard_hello")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("loiter_radius_m", 50.0)
        self.declare_parameter("alt_m", -50.0)  # NED: negative is up
        self.declare_parameter("speed_mps", 15.0)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.dt = 1.0 / rate_hz

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.t = 0.0
        self.setpoint_count = 0
        self.armed = False
        self.offboard_enabled = False

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info("offboard_hello started")

    def now_us(self) -> int:
        # PX4 messages typically use microseconds timestamps
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self, timestamp_us: int):
        msg = OffboardControlMode()
        msg.timestamp = timestamp_us
        # Use position control setpoints (NED)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, timestamp_us: int):
        radius = float(self.get_parameter("loiter_radius_m").value)
        alt = float(self.get_parameter("alt_m").value)

        # Simple circle in local NED around origin
        omega = 0.05  # rad/s
        x = radius * math.cos(omega * self.t)
        y = radius * math.sin(omega * self.t)
        z = alt

        msg = TrajectorySetpoint()
        msg.timestamp = timestamp_us
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)

        # yaw is optional; set to face tangentially
        msg.yaw = float(math.atan2(y, x) + math.pi / 2.0)

        self.traj_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def arm(self):
        # VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        self.publish_vehicle_command(400, param1=1.0)
        self.get_logger().info("Sent ARM")

    def set_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE = 176
        # PX4 main mode: 1 = custom, submode depends on PX4 version; common Offboard is custom mode.
        # Typical pattern:
        # param1 = 1 (PX4 custom mode)
        # param2 = 6 (OFFBOARD)  -- may vary by firmware; adjust if needed.
        self.publish_vehicle_command(176, param1=1.0, param2=6.0)
        self.get_logger().info("Sent OFFBOARD mode request")

    def on_timer(self):
        ts = self.now_us()

        # Always stream these
        self.publish_offboard_control_mode(ts)
        self.publish_trajectory_setpoint(ts)

        self.setpoint_count += 1
        self.t += self.dt

        # PX4 requires a short stream of setpoints before enabling offboard
        if self.setpoint_count == 40 and not self.offboard_enabled:
            self.set_offboard_mode()
            self.offboard_enabled = True

        if self.setpoint_count == 60 and not self.armed:
            self.arm()
            self.armed = True


def main():
    rclpy.init()
    node = OffboardHello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
