#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
    VehicleLocalPosition,
)

# -----------------------------
# Topics (match your ros2 topic list)
# -----------------------------
TOPIC_STATUS = "/fmu/out/vehicle_status_v1"
TOPIC_LPOS = "/fmu/out/vehicle_local_position_v1"
TOPIC_ACK = "/fmu/out/vehicle_command_ack"

TOPIC_OFFBOARD_MODE_IN = "/fmu/in/offboard_control_mode"
TOPIC_TRAJ_IN = "/fmu/in/trajectory_setpoint"
TOPIC_CMD_IN = "/fmu/in/vehicle_command"

# -----------------------------
# Offboard constants
# -----------------------------
SETPOINT_RATE_HZ = 20.0
SETPOINT_DT = 1.0 / SETPOINT_RATE_HZ

# PX4 requires a stream of setpoints before it will accept OFFBOARD
WARMUP_SECS = 1.0

# resend mode/arm requests periodically
MODE_REQUEST_PERIOD = 1.0
ARM_REQUEST_PERIOD = 1.0

# MAVLink command IDs used by PX4
VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400

# For VEHICLE_CMD_DO_SET_MODE:
# param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
# param2 = PX4 custom main mode (OFFBOARD=6)
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0


def now_us(node: Node) -> int:
    """PX4 msgs expect timestamps in microseconds."""
    return int(node.get_clock().now().nanoseconds // 1000)


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0


class OffboardController(Node):
    def __init__(self) -> None:
        super().__init__("offboard_controller")
        self.get_logger().info("Starting OffboardController")

        # PX4 uXRCE DDS topics are typically BEST_EFFORT.
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribers
        self.sub_status = self.create_subscription(
            VehicleStatus, TOPIC_STATUS, self.status_cb, qos_best_effort
        )
        self.sub_lpos = self.create_subscription(
            VehicleLocalPosition, TOPIC_LPOS, self.lpos_cb, qos_best_effort
        )
        self.sub_ack = self.create_subscription(
            VehicleCommandAck, TOPIC_ACK, self.ack_cb, qos_best_effort
        )

        # Publishers (publishing best effort is fine; PX4 side accepts it)
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, TOPIC_OFFBOARD_MODE_IN, qos_best_effort
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, TOPIC_TRAJ_IN, qos_best_effort
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, TOPIC_CMD_IN, qos_best_effort
        )

        # State
        self.status: Optional[VehicleStatus] = None
        self.lpos: Optional[VehicleLocalPosition] = None

        self.t0 = time.time()
        self.last_mode_req = 0.0
        self.last_arm_req = 0.0
        self.last_wait_log = 0.0

        self.warmup_start = time.time()
        self.sent_any_setpoint = False
        self.offboard_requested = False
        self.armed_requested = False

        # Simple “route”: a few waypoints in local NED
        # IMPORTANT: z is "down" in NED. Negative z means "up".
        # Tune these for your sim. For airplane, big jumps can be weird; keep reasonable.
        self.route: List[Waypoint] = [
            Waypoint(x=200.0, y=0.0, z=-50.0, yaw=0.0),
            Waypoint(x=400.0, y=100.0, z=-60.0, yaw=0.2),
            Waypoint(x=600.0, y=-100.0, z=-60.0, yaw=-0.2),
            Waypoint(x=800.0, y=0.0, z=-55.0, yaw=0.0),
            Waypoint(x=1000.0, y=0.0, z=-50.0, yaw=0.0),
        ]
        self.wp_idx = 0

        # Controller gains / thresholds for waypoint switching
        self.wp_accept_radius_m = 30.0  # airplane: keep larger than multicopter
        self.wp_accept_alt_m = 15.0

        # Timer: 20 Hz setpoint publishing + state machine
        self.timer = self.create_timer(SETPOINT_DT, self.timer_cb)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def status_cb(self, msg: VehicleStatus) -> None:
        self.status = msg

    def lpos_cb(self, msg: VehicleLocalPosition) -> None:
        self.lpos = msg

    def ack_cb(self, msg: VehicleCommandAck) -> None:
        # result=0 means accepted/success in PX4 ack mapping (often)
        self.get_logger().info(
            f"CMD_ACK: command={msg.command} result={msg.result} "
            f"target_system={msg.target_system} target_component={msg.target_component}"
        )

    # -----------------------------
    # Helpers
    # -----------------------------
    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = now_us(self)

        # For TrajectorySetpoint-based control
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.pub_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self, wp: Waypoint) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = now_us(self)

        # Your msg definition: float32[3] position/velocity/acceleration/jerk
        msg.position = [float(wp.x), float(wp.y), float(wp.z)]

        # Use NaN to indicate “don’t control” fields (per PX4 message comment).
        # For a simple position route, leave velocity/accel/jerk as NaN.
        nan = float("nan")
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]

        msg.yaw = float(wp.yaw)
        msg.yawspeed = 0.0

        self.pub_traj.publish(msg)

    def send_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = now_us(self)

        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)

        # param5/6 are double in your VehicleCommand definition
        msg.param5 = float(param5)
        msg.param6 = float(param6)

        msg.param7 = float(param7)

        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True

        self.pub_cmd.publish(msg)

    def dist_to_wp(self, wp: Waypoint) -> Optional[float]:
        if self.lpos is None:
            return None
        dx = wp.x - float(self.lpos.x)
        dy = wp.y - float(self.lpos.y)
        dz = wp.z - float(self.lpos.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def advance_waypoint_if_needed(self) -> None:
        if self.lpos is None:
            return
        if self.wp_idx >= len(self.route):
            return

        wp = self.route[self.wp_idx]

        dx = wp.x - float(self.lpos.x)
        dy = wp.y - float(self.lpos.y)
        dz = wp.z - float(self.lpos.z)

        horiz = math.sqrt(dx * dx + dy * dy)
        vert = abs(dz)

        if horiz < self.wp_accept_radius_m and vert < self.wp_accept_alt_m:
            self.get_logger().info(
                f"Reached WP[{self.wp_idx+1}/{len(self.route)}], advancing..."
            )
            self.wp_idx += 1
            if self.wp_idx >= len(self.route):
                self.get_logger().info("Route complete. Holding last waypoint.")

    # -----------------------------
    # Main loop
    # -----------------------------
    def timer_cb(self) -> None:
        t = time.time()

        # Wait until we see status + local position
        if self.status is None or self.lpos is None:
            if t - self.last_wait_log > 5.0:
                self.get_logger().info(
                    f"Waiting for {TOPIC_STATUS} and {TOPIC_LPOS} ... (BEST_EFFORT QoS)"
                )
                self.last_wait_log = t
            return

        # Optional: wait until PX4 says preflight checks passed
        # (You already got it passing with SYS_AUTOSTART=10041, so this should be true.)
        if not bool(self.status.pre_flight_checks_pass):
            if t - self.last_wait_log > 2.0:
                self.get_logger().warn(
                    "Preflight checks not passing yet. Not requesting arm/offboard."
                )
                self.last_wait_log = t
            # Still publish setpoints/mode so system is ready once checks pass
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(self.route[self.wp_idx])
            self.sent_any_setpoint = True
            return

        # Always publish control mode + setpoint at 20Hz
        self.publish_offboard_control_mode()

        # Route logic
        if self.wp_idx >= len(self.route):
            wp = self.route[-1]
        else:
            wp = self.route[self.wp_idx]
            self.advance_waypoint_if_needed()

        self.publish_trajectory_setpoint(wp)
        self.sent_any_setpoint = True

        # Periodic debug about route
        if t - self.last_wait_log > 5.0:
            d = self.dist_to_wp(wp)
            if d is not None:
                self.get_logger().info(
                    f"WP[{min(self.wp_idx+1,len(self.route))}/{len(self.route)}] "
                    f"target (x={wp.x:.1f}, y={wp.y:.1f}, z={wp.z:.1f}) "
                    f"pos (x={self.lpos.x:.1f}, y={self.lpos.y:.1f}, z={self.lpos.z:.1f}) "
                    f"dist={d:.1f}m"
                )
            self.last_wait_log = t

        # Warmup: keep sending setpoints before asking OFFBOARD
        if (t - self.warmup_start) < WARMUP_SECS:
            return

        # Request OFFBOARD mode periodically until accepted (PX4 will ACK if it wants)
        if (t - self.last_mode_req) > MODE_REQUEST_PERIOD:
            self.send_vehicle_command(
                VEHICLE_CMD_DO_SET_MODE,
                param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
                param3=0.0,
            )
            self.last_mode_req = t
            self.get_logger().info("Sent OFFBOARD mode request (VEHICLE_CMD_DO_SET_MODE)")
            self.offboard_requested = True

        # Request ARM periodically
        if (t - self.last_arm_req) > ARM_REQUEST_PERIOD:
            self.send_vehicle_command(
                VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,   # arm
                param2=0.0,   # no force
            )
            self.last_arm_req = t
            self.get_logger().info("Sent ARM command (VEHICLE_CMD_COMPONENT_ARM_DISARM)")
            self.armed_requested = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down OffboardController")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
