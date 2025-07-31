# mycobot_320_joy

This ROS 2 package provides a teleoperation node that lets you control the Elephant Robotics **myCobot 320 Pi** robot arm using an Xbox controller.  The package is targeted at the Galactic distribution of ROS 2 but should work on other distros with minor changes.

## Features

* Subscribes to `/joy` and converts joystick axes and button presses into joint angle increments for the six‑axis myCobot 320 Pi.
* Uses the [`pymycobot` Python API](https://github.com/elephantrobotics/pymycobot) to communicate with the robot over a serial port.
* Parameters allow you to configure the serial `port`, joystick `deadzone`, scaling factor (`scale`) and movement `speed`.
* Comes with a `launch/teleop.launch.py` file that starts both the standard ROS 2 `joy` node and this teleop node.

## Prerequisites

1. **Install pymycobot.**  The official mycobot_ros2 README notes that you must have the Python library installed to communicate with the arm【636979306340876†L274-L302】:
   ```bash
   pip install --user pymycobot
   ```
2. **Install the ROS 2 `joy` package.**  This provides the `joy_node` used to read the Xbox controller.  On Ubuntu you can install it with:
   ```bash
   sudo apt install ros-galactic-joy
   ```
   After installation, verify that your gamepad is detected by running the node and echoing the `/joy` topic【489178215549443†L46-L49】:
   ```bash
   ros2 run joy joy_node
   ros2 topic echo /joy
   ```
3. **ROS 2 workspace.**  You should already have a `colcon` workspace (e.g. `~/colcon_ws`).  If not, create one with:
   ```bash
   mkdir -p ~/colcon_ws/src
   cd ~/colcon_ws
   colcon build
   ```

## Installation

Clone this repository into the `src` directory of your ROS 2 workspace and build it using `colcon` as you would any other package.  The mycobot_ros2 README provides a similar set of instructions for building their packages【636979306340876†L274-L302】.

```bash
cd ~/colcon_ws/src
# clone this repository
git clone https://github.com/<your-username>/mycobot_320_joy.git

# build the workspace
cd ~/colcon_ws
colcon build --symlink-install

# source the overlay
source install/local_setup.bash
```

## Running

Launch the teleoperation demo using the provided launch file.  Make sure your Xbox controller is connected and recognized, and adjust the serial port for the myCobot if necessary (for example `/dev/ttyAMA0`, `/dev/ttyACM0` or `/dev/ttyUSB0`).

```bash
# source your workspace
source ~/colcon_ws/install/local_setup.bash

# start the joy and teleop nodes
ros2 launch mycobot_320_joy teleop.launch.py
```

If everything is configured correctly, moving the sticks on your Xbox controller should cause the myCobot arm to move.  The `B` button resets the arm to zero position by default.  You can tune parameters such as `scale` and `deadzone` via the launch file or by remapping parameters at runtime.

## Notes

* The default joystick mapping assumes a standard Xbox controller.  You can modify the axis mapping in `mycobot_joy.py` if your controller has a different layout.
* For remote control over the network, ROS 2 uses DDS discovery.  Consider using a VPN (e.g., Husarnet, ZeroTier, WireGuard) or a rosbridge/WebSocket bridge to relay `/joy` messages over the internet.
* Always ensure the robot has sufficient clearance during testing to prevent collisions.
