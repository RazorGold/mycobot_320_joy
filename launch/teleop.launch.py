"""Launch file to start the myCobot 320 Pi teleoperation using an Xbox controller.

This launch description starts the ROS2 `joy` node (from the `joy` package) to read
inputs from a gamepad and the custom `mycobot_joy_node` defined in the
mycobot_320_joy package to translate joystick movements into myCobot joint
commands.

You may need to adjust the `dev` parameter for the joystick device
(e.g. `/dev/input/js0`) and the serial port for the myCobot (e.g.
`/dev/ttyAMA0` or `/dev/ttyACM0`) to match your hardware.  The `scale` and
`deadzone` parameters can also be tuned via the teleop node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description to start joy and teleop nodes."""
    # Node to read inputs from the Xbox controller
    joy_node = Node(
        package='joy',
        executable='joy_node',  # for Galactic, the joy node executable is joy_node
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )

    # Teleop node converting joystick inputs into myCobot commands
    teleop_node = Node(
        package='mycobot_320_joy',
        executable='mycobot_joy_node',
        name='mycobot_joy_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyAMA0',  # adjust to match your myCobot 320 serial port (e.g. /dev/ttyACM0)
            'scale': 2.0,            # scaling factor for joystick input (degrees per unit input)
            'deadzone': 0.1          # joystick deadzone to ignore small movements
        }]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
