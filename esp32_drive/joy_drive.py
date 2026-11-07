#!/usr/bin/env python3
"""
joy_drive.py - Unrestricted Continuous ICR Controller with Live Encoder Latching
Controls:
  [w / q / +] : Increase Target Speed (+0.1 m/s) -- Does NOT start movement
  [x / z / -] : Decrease Target Speed (-0.1 m/s) -- Does NOT start movement
  [i / 3]     : Drive FRONT at Target Speed
  [, / s / 5] : Drive BACK at Target Speed
  [j / a / 4] : Shift ICR Left (Turn Left / Tighten Left Curvature)
  [l / d / 6] : Shift ICR Right (Turn Right / Tighten Right Curvature)
  [k / SPACE] : Stop movement (keeps current physical steering angle and target speed)
  [h]         : Home (Stop movement AND reset steering curvature to 0)
"""

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String

VALID_KEYS = [
    'i', ',', 'j', 'l', 's', 'a', 'd', '3', '4', '5', '6',
    'w', 'x', 'q', 'z', '+', '-', '=',
    'h', ' ', 'k'
]
WHEEL_ORDER = ["RF", "RR", "LF", "LR"]

# Window (in seconds) that a J/L keypress is considered actively held
STEER_HOLD_WINDOW = 0.15


class UnrestrictedCurvatureController(Node):
    def __init__(self):
        super().__init__('unrestricted_curvature_controller')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub_cmd = self.create_publisher(Float32MultiArray, '/joy_drive_cmd', 10)
        self.sub_source = self.create_subscription(String, '/drive_source', self.source_callback, 10)
        self.sub_enc = self.create_subscription(Float32MultiArray, '/pivot_encoders', self.enc_callback, sensor_qos)

        # Rover Dimensions (matching controller_sh.py)
        self.L = 0.7777  # Wheelbase (x)
        self.W = 0.4750  # Track Width (y)

        self.wheel_positions = {
            "RF": ( self.L / 2.0, -self.W / 2.0),
            "RR": (-self.L / 2.0, -self.W / 2.0),
            "LF": ( self.L / 2.0,  self.W / 2.0),
            "LR": (-self.L / 2.0,  self.W / 2.0),
        }

        # Separated Speed Slider & Drive State
        self.target_speed = 0.5
        self.drive_dir = 0.0        # 0.0 = Stopped, 1.0 = Forward, -1.0 = Backward
        self.curvature_param = 0.0  # < 0: Right turn, > 0: Left turn, 0: Straight

        # Live Encoder State (["RF", "RR", "LF", "LR"])
        self.enc_angles = [0.0, 0.0, 0.0, 0.0]
        self.enc_seen = False
        self.last_steer_press = None
        self.homing = False

        # Step & Limit Parameters
        self.lin_step = 0.1
        self.min_v = 0.1
        self.max_v = 2.0
        self.curvature_step = 0.05

        self.lock = threading.Lock()
        self.running = True
        self.key_seq = 0

        self.timer = self.create_timer(0.05, self.publish_state)
        self.kb_start_timer = self.create_timer(0.5, self.start_keyboard)

        self.get_logger().info(
            "ICR Controller Ready. Target angle latches to live encoder when [J/L] is not pressed."
        )

    def enc_callback(self, msg):
        if len(msg.data) >= 4:
            with self.lock:
                self.enc_angles = [float(a) for a in msg.data[:4]]
                self.enc_seen = True

    def source_callback(self, msg):
        parts = msg.data.split()
        if len(parts) != 2 or parts[0] != 'teleop':
            return
        try:
            echoed = int(parts[1])
        except ValueError:
            return
        if echoed != self.key_seq:
            return
        with self.lock:
            self.drive_dir = 0.0
            self.curvature_param = 0.0

    def curve_from_live_encoders(self, enc):
        """Backs out curvature_param from the current physical front wheel encoder angles."""
        avg_front_deg = 0.5 * (enc[0] + enc[2])
        if abs(avg_front_deg) < 0.5:
            return 0.0
        tan_val = math.tan(math.radians(avg_front_deg))
        if abs(tan_val) < 1e-4:
            return 0.0
        R_c = (self.L / 2.0) / tan_val
        sign = 1.0 if R_c >= 0.0 else -1.0
        c_mag = (self.L / 2.0) / (abs(R_c) + (self.L / 2.0))
        return sign * max(0.0, min(1.0, c_mag))

    def compute_wheel_trajectories(self, v, target_curve_param, speed_curve_param):
        # 1. Compute target angles from target_curve_param
        angles = {}
        if abs(target_curve_param) < 1e-4:
            R_c_target = float('inf')
            for name in WHEEL_ORDER:
                angles[name] = 0.0
        else:
            sign_t = 1.0 if target_curve_param > 0.0 else -1.0
            if abs(target_curve_param) < 1.0:
                R_c_target = sign_t * (self.L / 2.0) * ((1.0 - abs(target_curve_param)) / abs(target_curve_param))
            else:
                R_c_target = 0.0
            for name, (x_i, y_i) in self.wheel_positions.items():
                angles[name] = math.degrees(math.atan2(sign_t * x_i, sign_t * (R_c_target - y_i)))

        # 2. Compute wheel linear speeds from LIVE encoder curvature (speed_curve_param)
        speeds = {}
        if abs(speed_curve_param) < 1e-4:
            R_c_live = float('inf')
            for name in WHEEL_ORDER:
                speeds[name] = abs(v)
        else:
            sign_s = 1.0 if speed_curve_param > 0.0 else -1.0
            if abs(speed_curve_param) < 1.0:
                R_c_live = sign_s * (self.L / 2.0) * ((1.0 - abs(speed_curve_param)) / abs(speed_curve_param))
            else:
                R_c_live = 0.0

            max_r = 0.0
            radii = {}
            for name, (x_i, y_i) in self.wheel_positions.items():
                r_i = math.hypot(x_i, R_c_live - y_i)
                radii[name] = r_i
                if r_i > max_r:
                    max_r = r_i

            for name in WHEEL_ORDER:
                speeds[name] = abs(v) * (radii[name] / max_r) if max_r > 1e-6 else 0.0

        direction = -1.0 if v >= 0.0 else 1.0
        out_angles = []
        out_speeds = []

        for name in WHEEL_ORDER:
            out_angles.append(float(angles[name]))
            if abs(v) < 1e-5:
                out_speeds.append(0.0)
            else:
                side_sign = -1.0 if name in ("LF", "LR") else 1.0
                out_speeds.append(float(speeds[name] * direction * side_sign))

        return R_c_live, out_angles, out_speeds

    def start_keyboard(self):
        self.kb_start_timer.cancel()
        try:
            self._tty_fd = sys.stdin.fileno()
            self._tty_attrs = termios.tcgetattr(self._tty_fd)
            tty.setcbreak(self._tty_fd)
            termios.tcflush(self._tty_fd, termios.TCIFLUSH)
        except Exception as e:
            self._tty_attrs = None
            self.get_logger().error(f"Could not set up keyboard: {e}")
            return

        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def restore_terminal(self):
        attrs = getattr(self, '_tty_attrs', None)
        if attrs is not None:
            self._tty_attrs = None
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, attrs)
            except Exception:
                pass

    def keyboard_loop(self):
        try:
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([self._tty_fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._tty_fd, 1)
                if not data:
                    break
                key = data.decode(errors='ignore').lower()
                if key in VALID_KEYS:
                    self.handle_key(key)
        except Exception as e:
            print(f"\nKeyboard error: {e}")
        finally:
            self.running = False
            self.restore_terminal()
            if rclpy.ok():
                rclpy.shutdown()

    def handle_key(self, key):
        with self.lock:
            now = self.get_clock().now()
            self.key_seq = (self.key_seq + 1) % 1000000

            if key in ('w', 'q', '+', '='):
                self.target_speed = min(self.max_v, round(self.target_speed + self.lin_step, 2))
            elif key in ('x', 'z', '-'):
                self.target_speed = max(self.min_v, round(self.target_speed - self.lin_step, 2))
            elif key in ('i', '3'):
                self.drive_dir = 1.0
            elif key in (',', 's', '5'):
                self.drive_dir = -1.0
            elif key in ('j', 'a', '4'):
                self.homing = False
                self.last_steer_press = now
                self.curvature_param = min(1.0, round(self.curvature_param + self.curvature_step, 3))
            elif key in ('l', 'd', '6'):
                self.homing = False
                self.last_steer_press = now
                self.curvature_param = max(-1.0, round(self.curvature_param - self.curvature_step, 3))
            elif key in ('k', ' '):
                self.drive_dir = 0.0
            elif key == 'h':
                self.drive_dir = 0.0
                self.curvature_param = 0.0
                self.homing = True

    def stop_and_publish(self):
        with self.lock:
            self.drive_dir = 0.0
            held_angles = list(self.enc_angles) if self.enc_seen else [0.0] * 4
        msg = Float32MultiArray()
        msg.data = held_angles + [0.0] * 4 + [float(self.key_seq)]
        self.pub_cmd.publish(msg)

    def publish_state(self):
        with self.lock:
            now = self.get_clock().now()
            active_v = self.target_speed * self.drive_dir

            steering_active = (
                self.last_steer_press is not None and
                ((now - self.last_steer_press).nanoseconds / 1e9) <= STEER_HOLD_WINDOW
            )

            live_curve = self.curve_from_live_encoders(self.enc_angles) if self.enc_seen else self.curvature_param

            # When J/L is not pressed (and not homing), make live encoder feedback the target angle directly
            if not steering_active and not self.homing and self.enc_seen:
                self.curvature_param = live_curve

            R_c, ideal_angles, speeds = self.compute_wheel_trajectories(
                active_v, self.curvature_param, live_curve
            )

            if not steering_active and not self.homing and self.enc_seen:
                angles = [float(a) for a in self.enc_angles]
            else:
                angles = ideal_angles

            msg = Float32MultiArray()
            msg.data = angles + speeds + [float(self.key_seq)]
            self.pub_cmd.publish(msg)

            r_str = "   INF" if math.isinf(R_c) else f"{R_c:6.2f}m"
            state_str = "FWD" if self.drive_dir > 0 else ("REV" if self.drive_dir < 0 else "STP")
            print(
                f"\rState: {state_str} | "
                f"SetV: {self.target_speed:4.2f}m/s (Act: {active_v:5.2f}) | "
                f"Rc: {r_str} | "
                f"LF:{angles[2]:6.1f}° RF:{angles[0]:6.1f}°   ",
                end="",
                flush=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = UnrestrictedCurvatureController()
    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.stop_and_publish()
        except Exception:
            pass
        node.running = False
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()