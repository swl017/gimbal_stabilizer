#!/usr/bin/env python3
"""Keyboard teleop for gimbal LOS rate control.

Terminal-based (curses) node that publishes normalized LOS rate commands
to gimbal_cmd_los_rate (geometry_msgs/Vector3).

Key mapping (matching Isaac Lab teleop):
    Z / X  ->  azimuth  +/-
    T / G  ->  elevation +/-
    Q / ESC -> quit

Hold-to-move: rate is non-zero only while key is held.
"""

import curses
import threading

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class GimbalTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('gimbal_teleop_keyboard')

        # Parameters
        self.declare_parameter('az_sensitivity', 0.3)
        self.declare_parameter('el_sensitivity', 0.3)
        self.declare_parameter('max_vel', 360.0)
        self.declare_parameter('publish_rate', 50.0)

        self.az_sens = self.get_parameter('az_sensitivity').value
        self.el_sens = self.get_parameter('el_sensitivity').value
        self.max_vel = self.get_parameter('max_vel').value
        pub_rate = self.get_parameter('publish_rate').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Vector3, 'gimbal_cmd_los_rate', qos)

        self.az_rate = 0.0
        self.el_rate = 0.0
        self._lock = threading.Lock()
        self._quit = False

        self.timer = self.create_timer(1.0 / pub_rate, self._publish)

    def _publish(self):
        msg = Vector3()
        with self._lock:
            msg.x = self.az_rate * self.max_vel
            msg.y = self.el_rate * self.max_vel
            msg.z = 0.0
        self.pub.publish(msg)

    def set_rates(self, az: float, el: float):
        with self._lock:
            self.az_rate = az
            self.el_rate = el

    def request_quit(self):
        self._quit = True


def curses_loop(stdscr, node: GimbalTeleopKeyboard):
    """Read keyboard input via curses and update node rates."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(20)  # 50 Hz poll

    stdscr.addstr(0, 0, '=== Gimbal Teleop Keyboard ===')
    stdscr.addstr(2, 0, 'Key bindings:')
    stdscr.addstr(3, 0, '  Z / X  : azimuth  +/-')
    stdscr.addstr(4, 0, '  T / G  : elevation +/-')
    stdscr.addstr(5, 0, '  Q / ESC: quit')
    stdscr.addstr(7, 0, f'az_sensitivity: {node.az_sens}')
    stdscr.addstr(8, 0, f'el_sensitivity: {node.el_sens}')
    stdscr.addstr(9, 0, f'max_vel: {node.max_vel} deg/s')
    stdscr.addstr(11, 0, 'Hold keys to slew. Release to stop.')

    # Track which keys are "held" — curses gives key-repeat events
    # but no explicit key-release. We zero the rate each cycle and
    # re-set it if the key is still being pressed (via key-repeat).
    while not node._quit:
        az = 0.0
        el = 0.0

        # Drain all pending keys this cycle
        while True:
            ch = stdscr.getch()
            if ch == -1:
                break
            if ch in (ord('q'), ord('Q'), 27):  # Q or ESC
                node.request_quit()
                return
            if ch in (ord('z'), ord('Z')):
                az += node.az_sens
            if ch in (ord('x'), ord('X')):
                az -= node.az_sens
            if ch in (ord('t'), ord('T')):
                el += node.el_sens
            if ch in (ord('g'), ord('G')):
                el -= node.el_sens

        # Clamp
        az = max(-1.0, min(1.0, az))
        el = max(-1.0, min(1.0, el))
        node.set_rates(az, el)

        # Status line
        try:
            stdscr.move(13, 0)
            stdscr.clrtoeol()
            stdscr.addstr(13, 0, f'az: {az:+.2f}  el: {el:+.2f}')
        except curses.error:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTeleopKeyboard()

    # Spin ROS in a background thread so curses owns the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        curses.wrapper(lambda stdscr: curses_loop(stdscr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
