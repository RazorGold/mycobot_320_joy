#!/usr/bin/env python3
"""
ROS 2 node to teleoperate an Elephant Robotics myCobot 320 Pi using a gamepad.
This node subscribes to the standard `/joy` topic and converts joystick axes into
incremental joint angle commands, then sends those angles to the robot via
pymycobot. It is designed for an Xbox controller on ROS 2 Galactic but can be
reconfigured via parameters for different controllers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    # Attempt to import the high‑level MyCobot class for myCobot 320
    from pymycobot.mycobot import MyCobot
except Exception:
    # Fallback to generic import if submodule layout differs
    from pymycobot import MyCobot


class MyCobotJoyTeleop(Node):
    """ROS2 node that teleoperates a myCobot 320 Pi using a gamepad."""

    def __init__(self) -> None:
        super().__init__('mycobot_joy_teleop')

        # Declare parameters with sensible defaults. These can be overridden
        # via the launch file or command line when running the node.
        self.declare_parameter('device', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('joint_speed', 30)  # Percent (0‑100) speed for send_angles
        self.declare_parameter('scale', 5.0)       # Degrees to increment per full stick deflection
        self.declare_parameter('deadzone', 0.1)    # Ignore small joystick movements

        device = self.get_parameter('device').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Try to connect to the robot
        try:
            self.mc = MyCobot(device, baudrate)
            self.get_logger().info(f'Connected to myCobot on {device} at {baudrate} bps.')
        except Exception as exc:
            self.get_logger().error(f'Failed to connect to myCobot: {exc}')
            raise

        # Read the robot's current angles as the starting pose
        try:
            angles = self.mc.get_angles()
            if not angles or len(angles) != 6:
                angles = [0.0] * 6
        except Exception:
            angles = [0.0] * 6
        self.current_angles = list(angles)

        # Subscribe to joystick messages
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Map joystick axes to myCobot joints. This mapping is based on the
        # default Xbox controller mapping from the `joy` package. Feel free to
        # adjust these indices to suit your own controller layout.
        #   0: left stick horizontal -> joint 1 (base rotation)
        #   1: left stick vertical   -> joint 2 (shoulder)
        #   3: right stick vertical  -> joint 3 (elbow)
        #   4: right stick horizontal-> joint 4 (wrist angle)
        #   2: left trigger          -> joint 5 (wrist rotation)
        #   5: right trigger         -> joint 6 (wrist roll)
        self.axis_joint_map = {
            0: 0,
            1: 1,
            3: 2,
            4: 3,
            2: 4,
            5: 5,
        }

    def joy_callback(self, msg: Joy) -> None:
        """Process joystick input and update myCobot joint angles."""
        axes = msg.axes
        buttons = msg.buttons
        scale = self.get_parameter('scale').get_parameter_value().double_value
        deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        speed = self.get_parameter('joint_speed').get_parameter_value().integer_value

        changed = False

        # Update each joint based on its mapped axis
        for axis_index, joint_idx in self.axis_joint_map.items():
            if axis_index >= len(axes):
                continue
            value = axes[axis_index]
            # Triggers on many controllers have a range of [‑1,1] where unpressed
            # may correspond to 1 and fully pressed corresponds to ‑1. Invert
            # triggers so pulling the trigger produces a positive increment.
            if axis_index in (2, 5):
                value = -value
            # Apply deadzone
            if abs(value) > deadzone:
                self.current_angles[joint_idx] += value * scale
                changed = True

        # Example reset: pressing the B button (button index 1) resets all joints
        if len(buttons) > 1 and buttons[1]:
            self.get_logger().info('Resetting robot pose to zero.')
            self.current_angles = [0.0] * 6
            changed = True

        if changed:
            # Clamp each angle to [‑180, 180] to avoid exceeding myCobot limits
            self.current_angles = [max(min(angle, 180.0), ‑180.0) for angle in self.current_angles]
            try:
                self.mc.send_angles(self.current_angles, speed)
            except Exception as exc:
                self.get_logger().error(f'Error sending angles: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MyCobotJoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
