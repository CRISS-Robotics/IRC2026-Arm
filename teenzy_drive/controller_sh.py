#!/usr/bin/env python3
"""
controller_sh.py - Unified ICR Curvature & AutoNav Swerve Controller (Teensy 4.1)
- Converts /cmd_vel (Nav2 / Teleop) into exact Instantaneous Center of Rotation (ICR)
  trajectories + separate 90-deg Crab mode.
- Matches Teensy 4.1 hardware drive polarity (DRIVE_POLARITY = -1.0 -> forward).
- Eliminates inner-wheel slip when |R_c| < W/2 (0.2375 m).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

L = 0.7777
W = 0.4750

# Matches teensy_drive_pivot.ino hardware offsets
ZERO_DEG_OFFSET = [2744, 3321, 3911, 214]
ENC_TICKS_PER_REV = 4096.0

# Hardware polarity: in joy_drive_3.py, drive_dir = -1.0 drives the rover FORWARD.
# Keeping DRIVE_POLARITY = -1.0 ensures positive cmd_vel.linear.x (AutoNav forward)
# sends negative wheel_speeds just like manual 'i' (FRONT).
DRIVE_POLARITY = -1.0

CMD_VEL_TIMEOUT_SEC = 0.3
JOY_CMD_TIMEOUT_SEC = 0.3
PUBLISH_RATE_HZ = 20.0
MAX_WHEEL_SPEED = 1.0
MAX_LINEAR_ACCEL = 1.0     # m/s^2
MAX_ANGULAR_ACCEL = 2.0    # rad/s^2

# --- Pivot Anti-Slip Parameters ---
PIVOT_SLEW_RATE_DEG_PER_SEC = 120.0
STEER_WAIT_THRESHOLD_DEG = 20.0
STEER_DEADBAND_DEG = 2.0
USE_HARDWARE_ENCODERS = True

WHEEL_POSITIONS = {
    "RF": ( L / 2.0, -W / 2.0),
    "RR": (-L / 2.0, -W / 2.0),
    "LF": ( L / 2.0,  W / 2.0),
    "LR": (-L / 2.0,  W / 2.0),
}

WHEEL_ORDER = ["RF", "RR", "LF", "LR"]


def normalize_angle(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg


def ticks_to_deg(raw_tick, zero_offset):
    diff_ticks = (raw_tick - zero_offset) % ENC_TICKS_PER_REV
    deg = (diff_ticks / ENC_TICKS_PER_REV) * 360.0
    return normalize_angle(deg)


def ramp_value(current, target, max_delta):
    if max_delta <= 0.0:
        return target
    if target > current:
        return min(target, current + max_delta)
    return max(target, current - max_delta)


def compute_icr_from_twist(vx, vy, wz, last_angles, max_wheel_speed):
    """
    Computes slip-free wheel angles and normalized wheel speeds from Twist (vx, vy, wz)
    using the exact same Radius of Curvature (R_c) geometry as joy_drive.py,
    plus automatic 90-degree crabbing when pure vy is commanded.
    """
    # 1. Stopped: hold last steering angles, zero wheel speeds
    if abs(vx) < 1e-4 and abs(vy) < 1e-4 and abs(wz) < 1e-4:
        return [last_angles[name] for name in WHEEL_ORDER], [0.0] * 4

    # 2. Separate 90-Degree Crab Mode (pure lateral vy command)
    if abs(vy) >= 1e-4 and abs(vx) < 1e-4 and abs(wz) < 1e-4:
        crab_spd = min(1.0, abs(vy) / max_wheel_speed)
        # +vy is left (+90 deg forward spin), -vy is right (+90 deg reverse spin)
        crab_dir = (1.0 if vy > 0.0 else -1.0) * DRIVE_POLARITY
        return [90.0] * 4, [crab_spd * crab_dir] * 4

    # 3. Straight-line driving (R_c = INF)
    if abs(wz) < 1e-4:
        lin_spd = min(1.0, abs(vx) / max_wheel_speed)
        lin_dir = (1.0 if vx >= 0.0 else -1.0) * DRIVE_POLARITY
        return [0.0] * 4, [lin_spd * lin_dir] * 4

    # 4. ICR Curvature Driving (including in-place rotation when vx == 0)
    if abs(vx) < 1e-4:
        R_c = 0.0
        sign = 1.0 if wz > 0.0 else -1.0
        # Tangential speed at wheel radius r_i = |wz| * r_i
        r_corner = math.hypot(L / 2.0, W / 2.0)
        v_mag = min(max_wheel_speed, abs(wz) * r_corner)
        forward_sign = 1.0
    else:
        # R_c = vx / wz (y-coordinate of ICR in rover frame)
        R_c = vx / wz
        sign = 1.0 if R_c > 0.0 else -1.0
        forward_sign = 1.0 if vx >= 0.0 else -1.0

    max_r = 0.0
    radii = {}
    angles = {}
    wheel_dirs = {}

    for name in WHEEL_ORDER:
        x_i, y_i = WHEEL_POSITIONS[name]
        dy = R_c - y_i
        r_i = math.hypot(x_i, dy)
        radii[name] = r_i
        if r_i > max_r:
            max_r = r_i

        # proj >= 0 when wheel is outside ICR, < 0 when ICR is inside track width (|R_c| < W/2)
        proj = sign * dy
        if abs(proj) < 1e-6:
            angles[name] = 90.0 if (sign * x_i) >= 0.0 else -90.0
            wheel_dirs[name] = 1.0
        else:
            angles[name] = math.degrees(math.atan2(sign * x_i, abs(proj)))
            wheel_dirs[name] = 1.0 if proj >= 0.0 else -1.0

    if abs(vx) >= 1e-4:
        # Scale so the fastest outer wheel matches the commanded linear/angular twist
        v_outer = abs(wz) * max_r
        v_mag = min(max_wheel_speed, max(abs(vx), v_outer))

    out_angles = []
    out_speeds = []
    base_dir = forward_sign * DRIVE_POLARITY

    for name in WHEEL_ORDER:
        out_angles.append(float(angles[name]))
        if max_r < 1e-6:
            out_speeds.append(0.0)
        else:
            spd_norm = (v_mag / max_wheel_speed) * (radii[name] / max_r)
            spd_norm = max(-1.0, min(1.0, spd_norm))
            out_speeds.append(float(spd_norm * base_dir * wheel_dirs[name]))

    return out_angles, out_speeds


class SwerveKinematicsNode(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_node')

        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED)
        self.declare_parameter('cmd_vel_timeout', CMD_VEL_TIMEOUT_SEC)
        self.declare_parameter('max_linear_accel', MAX_LINEAR_ACCEL)
        self.declare_parameter('max_angular_accel', MAX_ANGULAR_ACCEL)
        self.declare_parameter('joy_cmd_timeout', JOY_CMD_TIMEOUT_SEC)

        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.joy_timeout = self.get_parameter('joy_cmd_timeout').value

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_update_time = self.get_clock().now()

        self.last_angles = {name: 0.0 for name in WHEEL_ORDER}
        self.est_angles = [0.0] * 4

        self.enc_angles = [0.0] * 4
        self.enc_wheel_valid = [False] * 4
        self.last_enc_time = self.get_clock().now()

        self.cmd_moving = False
        self.joy_angles = [0.0] * 4
        self.joy_speeds = [0.0] * 4
        self.joy_seen = False
        self.joy_seq = None
        self.last_joy_time = self.get_clock().now()
        self.source = 'teleop'
        self._logged_source = self.source
        self._src_key = None
        self._src_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_joy = self.create_subscription(Float32MultiArray, '/joy_drive_cmd', self.joy_callback, 10)
        self.sub_enc = self.create_subscription(Float32MultiArray, '/pivot_encoders', self.enc_callback, sensor_qos)

        self.pub_angles = self.create_publisher(Float32MultiArray, '/target_angles', 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.pub_source = self.create_publisher(String, '/drive_source', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_wheel_commands)

        self.get_logger().info(
            f'swerve_kinematics_node ready at {PUBLISH_RATE_HZ} Hz | '
            f'ICR + AutoNav active | offsets={ZERO_DEG_OFFSET}'
        )

    def enc_callback(self, msg):
        if len(msg.data) >= 4:
            for i in range(4):
                val = float(msg.data[i])
                if 0.0 <= val < ENC_TICKS_PER_REV:
                    self.enc_angles[i] = ticks_to_deg(val, ZERO_DEG_OFFSET[i])
                    self.enc_wheel_valid[i] = True
                else:
                    self.enc_wheel_valid[i] = False
            self.last_enc_time = self.get_clock().now()

    def update_pivot_and_get_scale(self, target_angles_deg, dt, now):
        max_step = PIVOT_SLEW_RATE_DEG_PER_SEC * dt
        for i in range(4):
            diff = normalize_angle(target_angles_deg[i] - self.est_angles[i])
            if abs(diff) <= max_step:
                self.est_angles[i] = target_angles_deg[i]
            else:
                self.est_angles[i] = normalize_angle(
                    self.est_angles[i] + math.copysign(max_step, diff)
                )

        enc_age = (now - self.last_enc_time).nanoseconds / 1e9
        enc_fresh = USE_HARDWARE_ENCODERS and (enc_age < 0.5)

        min_scale = 1.0
        for i in range(4):
            target = target_angles_deg[i]
            est_err = abs(normalize_angle(target - self.est_angles[i]))

            if enc_fresh and self.enc_wheel_valid[i]:
                hw_err = abs(normalize_angle(target - self.enc_angles[i]))
                err = min(hw_err, est_err)
            else:
                err = est_err

            if err >= STEER_WAIT_THRESHOLD_DEG:
                return 0.0
            elif err <= STEER_DEADBAND_DEG:
                scale = 1.0
            else:
                norm_err = (err - STEER_DEADBAND_DEG) / (STEER_WAIT_THRESHOLD_DEG - STEER_DEADBAND_DEG)
                scale = math.cos(norm_err * (math.pi / 2.0))

            if scale < min_scale:
                min_scale = scale

        return min_scale

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.cmd_moving = any(abs(v) > 1e-6 for v in (self.vx, self.vy, self.wz))
        self.last_cmd_time = self.get_clock().now()
        if self.cmd_moving:
            self.source = 'teleop'

    def joy_callback(self, msg):
        if len(msg.data) < 9:
            return
        self.joy_angles = [float(a) for a in msg.data[:4]]
        self.joy_speeds = [float(s) for s in msg.data[4:8]]
        self.joy_seen = True
        self.last_joy_time = self.get_clock().now()
        seq = int(round(msg.data[8]))
        if seq != self.joy_seq:
            self.joy_seq = seq
            self.source = 'joy'

    def publish_source(self, now):
        key = f"{self.source} {-1 if self.joy_seq is None else self.joy_seq}"
        if key != self._src_key or (now - self._src_time).nanoseconds / 1e9 > 1.0:
            msg = String()
            msg.data = key
            self.pub_source.publish(msg)
            self._src_key = key
            self._src_time = now

    def publish_wheel_commands(self):
        now = self.get_clock().now()

        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / PUBLISH_RATE_HZ

        age = (now - self.last_cmd_time).nanoseconds / 1e9
        joy_age = (now - self.last_joy_time).nanoseconds / 1e9
        joy_fresh = self.joy_seen and joy_age <= self.joy_timeout

        if self.source != self._logged_source:
            self.get_logger().info(f'drive source: {self._logged_source} -> {self.source}')
            self._logged_source = self.source
        self.publish_source(now)

        if self.source == 'joy':
            self.current_vx = self.current_vy = self.current_wz = 0.0
            if joy_fresh:
                angles_deg = list(self.joy_angles)
                raw_speeds = [max(-1.0, min(1.0, s)) for s in self.joy_speeds]
                for name, angle in zip(WHEEL_ORDER, angles_deg):
                    self.last_angles[name] = angle

                align_scale = self.update_pivot_and_get_scale(angles_deg, dt, now)
                speeds_norm = [s * align_scale for s in raw_speeds]
            else:
                angles_deg = [self.last_angles[name] for name in WHEEL_ORDER]
                self.update_pivot_and_get_scale(angles_deg, dt, now)
                speeds_norm = [0.0] * 4
        else:
            if age > self.cmd_vel_timeout:
                target_vx, target_vy, target_wz = 0.0, 0.0, 0.0
            else:
                target_vx, target_vy, target_wz = self.vx, self.vy, self.wz

            max_lin_delta = self.max_linear_accel * dt
            max_ang_delta = self.max_angular_accel * dt
            self.current_vx = ramp_value(self.current_vx, target_vx, max_lin_delta)
            self.current_vy = ramp_value(self.current_vy, target_vy, max_lin_delta)
            self.current_wz = ramp_value(self.current_wz, target_wz, max_ang_delta)

            angles_deg, raw_speeds = compute_icr_from_twist(
                self.current_vx, self.current_vy, self.current_wz,
                self.last_angles, self.max_wheel_speed
            )
            for name, angle in zip(WHEEL_ORDER, angles_deg):
                self.last_angles[name] = angle

            align_scale = self.update_pivot_and_get_scale(angles_deg, dt, now)
            speeds_norm = [s * align_scale for s in raw_speeds]

        angles_msg = Float32MultiArray()
        angles_msg.data = [float(a) for a in angles_deg]
        self.pub_angles.publish(angles_msg)

        wheels_msg = Float32MultiArray()
        wheels_msg.data = [float(s) for s in speeds_norm] + [0.0]
        self.pub_wheels.publish(wheels_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()