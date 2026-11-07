#!/usr/bin/env python3
"""
joy_drive.py - Continuous ICR Controller + Separate 90° Crab Mode for Teensy 4.1
Controls:
  [w / q / +] : Increase Target Speed (+0.1 m/s) -- Does NOT start movement
  [x / z / -] : Decrease Target Speed (-0.1 m/s) -- Does NOT start movement
  [i / 3]     : Drive FRONT (or Crab LEFT when in 90° Crab Mode)
  [, / s / 5] : Drive BACK  (or Crab RIGHT when in 90° Crab Mode)
  [j / a / 4] : Shift ICR Left  (Turn Left / Tighten Left Curvature)
  [l / d / 6] : Shift ICR Right (Turn Right / Tighten Right Curvature)
  [c]         : Toggle Separate 90° CRAB Mode (locks all 4 wheels at +90°)
  [k / SPACE] : Stop movement (keeps current steering and target speed)
  [h]         : Home (Stop movement, exit Crab mode, reset curvature to 0)
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
from std_msgs.msg import Float32MultiArray, String

VALID_KEYS = [
    'i', ',', 'j', 'l', 's', 'a', 'd', '3', '4', '5', '6',
    'w', 'x', 'q', 'z', '+', '-', '=',
    'c', 'h', ' ', 'k'
]
WHEEL_ORDER = ["RF", "RR", "LF", "LR"]

# Matches your working manual direction in joy_drive_3.py where -1.0 drives forward
DRIVE_POLARITY = -1.0


class UnrestrictedCurvatureController(Node):
    def __init__(self):
        super().__init__('unrestricted_curvature_controller')

        self.pub_cmd = self.create_publisher(Float32MultiArray, '/joy_drive_cmd', 10)
        self.sub_source = self.create_subscription(String, '/drive_source', self.source_callback, 10)

        self.L = 0.7777  # Wheelbase (x)
        self.W = 0.4750  # Track Width (y)

        self.wheel_positions = {
            "RF": ( self.L / 2.0, -self.W / 2.0),
            "RR": (-self.L / 2.0, -self.W / 2.0),
            "LF": ( self.L / 2.0,  self.W / 2.0),
            "LR": (-self.L / 2.0,  self.W / 2.0),
        }

        self.target_speed = 0.5
        self.drive_dir = 0.0        # 0.0 = Stopped, +1.0 = Forward/Left, -1.0 = Backward/Right
        self.curvature_param = 0.0  # < 0: Right turn, > 0: Left turn, 0: Straight
        self.crab_mode = False      # Separate 90-degree crab mode

        self.lin_step = 0.1
        self.min_v = 0.1
        self.max_v = 1.0
        self.curvature_step = 0.05

        self.lock = threading.Lock()
        self.running = True

        self.key_seq = 0
        self.last_press = None
        self.settle_s = 1.0

        self.timer = self.create_timer(0.05, self.publish_state)
        self.kb_start_timer = self.create_timer(0.5, self.start_keyboard)

        self.get_logger().info(
            "ICR + 90° Crab Controller Ready. [W/X] Speed | [I/,] Move | [J/L] Steer | [C] 90° Crab | [K] Stop"
        )

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
            self.crab_mode = False

    def compute_wheel_trajectories(self, v, curve_param, crab_mode):
        out_angles = []
        out_speeds = []
        base_dir = (1.0 if v >= 0.0 else -1.0) * DRIVE_POLARITY

        # 1. Separate 90-Degree Crab Mode
        if crab_mode:
            for _ in WHEEL_ORDER:
                out_angles.append(90.0)
                out_speeds.append(0.0 if abs(v) < 1e-5 else float(abs(v) * base_dir))
            return float('nan'), out_angles, out_speeds

        # 2. Straight Line (R_c = INF)
        if abs(curve_param) < 1e-4:
            for _ in WHEEL_ORDER:
                out_angles.append(0.0)
                out_speeds.append(0.0 if abs(v) < 1e-5 else float(abs(v) * base_dir))
            return float('inf'), out_angles, out_speeds

        # 3. Continuous ICR Curvature
        sign = 1.0 if curve_param > 0.0 else -1.0
        if abs(curve_param) < 1.0:
            R_c = sign * (self.L / 2.0) * ((1.0 - abs(curve_param)) / abs(curve_param))
        else:
            R_c = 0.0

        max_r = 0.0
        radii = {}
        angles = {}
        wheel_dirs = {}

        for name, (x_i, y_i) in self.wheel_positions.items():
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
                # Smoothly steers in [-90, +90] and reverses inner wheel spin when |R_c| < W/2
                angles[name] = math.degrees(math.atan2(sign * x_i, abs(proj)))
                wheel_dirs[name] = 1.0 if proj >= 0.0 else -1.0

        for name in WHEEL_ORDER:
            out_angles.append(float(angles[name]))
            if abs(v) < 1e-5 or max_r < 1e-6:
                out_speeds.append(0.0)
            else:
                spd = abs(v) * (radii[name] / max_r)
                out_speeds.append(float(spd * base_dir * wheel_dirs[name]))

        return R_c, out_angles, out_speeds

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
            self.last_press = now

            if key in ('w', 'q', '+', '='):
                self.target_speed = min(self.max_v, round(self.target_speed + self.lin_step, 2))
            elif key in ('x', 'z', '-'):
                self.target_speed = max(self.min_v, round(self.target_speed - self.lin_step, 2))
            elif key in ('i', '3'):
                self.drive_dir = 1.0
            elif key in (',', 's', '5'):
                self.drive_dir = -1.0
            elif key in ('j', 'a', '4'):
                self.crab_mode = False
                self.curvature_param = min(1.0, round(self.curvature_param + self.curvature_step, 3))
            elif key in ('l', 'd', '6'):
                self.crab_mode = False
                self.curvature_param = max(-1.0, round(self.curvature_param - self.curvature_step, 3))
            elif key == 'c':
                self.crab_mode = not self.crab_mode
                self.drive_dir = 0.0
            elif key in ('k', ' '):
                self.drive_dir = 0.0
            elif key == 'h':
                self.drive_dir = 0.0
                self.curvature_param = 0.0
                self.crab_mode = False

    def stop_and_publish(self):
        with self.lock:
            self.drive_dir = 0.0
            self.curvature_param = 0.0
            self.crab_mode = False
        msg = Float32MultiArray()
        msg.data = [0.0] * 8 + [float(self.key_seq)]
        self.pub_cmd.publish(msg)

    def publish_state(self):
        with self.lock:
            active_v = self.target_speed * self.drive_dir
            R_c, angles, speeds = self.compute_wheel_trajectories(
                active_v, self.curvature_param, self.crab_mode
            )

            idle = (abs(active_v) < 1e-6 and abs(self.curvature_param) < 1e-6 and not self.crab_mode)
            now = self.get_clock().now()
            recent = (self.last_press is not None and (now - self.last_press).nanoseconds / 1e9 < self.settle_s)

            if not idle or recent:
                msg = Float32MultiArray()
                msg.data = angles + speeds + [float(self.key_seq)]
                self.pub_cmd.publish(msg)

            if self.crab_mode:
                r_str = "CRAB90"
                state_str = "CRB" if abs(self.drive_dir) > 0 else "STP"
            else:
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