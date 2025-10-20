# ISAAC ROS Workspace

This workspace contains ROS 2 packages for robotics development with NVIDIA ISAAC ROS.

## Recent Updates to ddsm_mobile_bringup Package

The `ddsm_mobile_bringup` package has been updated from ROS 2 Foxy to ROS 2 Humble with the following changes:

### 1. Package Configuration Updates

- Updated `package.xml` with Humble-compatible dependencies
- Modified `CMakeLists.txt` for Humble build system compatibility

### 2. Launch Files

- Updated `display.launch.py` for Humble compatibility
- Created `gazebo.launch.py` for Gazebo simulation support
- Created `teleop.launch.py` for gamepad control support

### 3. URDF Model

- Updated `simulation.urdf.xacro` with Gazebo plugin support
- Configured differential drive controller for Gazebo integration
- Restored robot model definition including base link and four wheels

### 4. Installation Commands

To install the required dependencies for Gazebo and teleop control:

```bash
# Install Ignition Gazebo and ROS 2 integration packages
sudo apt update
sudo apt install -y ignition-fortress ros-humble-ros-ign

# Install teleop packages for gamepad control
sudo apt install -y ros-humble-teleop-twist-joy ros-humble-teleop-tools

# Install additional dependencies
sudo apt install -y ros-humble-gazebo-dev ros-humble-gazebo-msgs \
    ros-humble-ignition-cmake2-vendor ros-humble-ignition-math6-vendor \
    ros-humble-diff-drive-controller ros-humble-ros-ign-gazebo-demos
```

### 5. Build Process

```bash
# Install required Python packages
pip3 install catkin_pkg

# Build the package
cd /workspaces/isaac_ros-dev/isaac_ros_ws
colcon build --packages-select ddsm_mobile_bringup

# Source the workspace
source install/setup.bash
```

### 6. Usage

After installation and building, you can run the following commands:

```bash
# Launch Gazebo simulation
ros2 launch ddsm_mobile_bringup gazebo.launch.py

# In another terminal, launch gamepad control
ros2 launch ddsm_mobile_bringup teleop.launch.py
```

### 7. Joystick Control Configuration

A joystick configuration file has been added to map gamepad controls:

- Axis 1: Controls forward/backward movement
- Axis 0: Controls left/right rotation  
- Button 6: Enable button for joystick control

The configuration is defined in `config/joystick.yaml` and can be launched with:

```bash
ros2 launch ddsm_mobile_bringup teleop.launch.py
```

This will start the joy_node for gamepad input and teleop_twist_joy_node for converting joystick inputs to robot commands.

### 8. Verification

To verify the package is correctly installed:

```bash
# Check if package is available
ros2 pkg list | grep ddsm_mobile_bringup
```

## Troubleshooting

If you encounter build errors related to missing Python modules:

```bash
# Install missing catkin_pkg module
pip3 install catkin_pkg
```

If Gazebo packages are not found, make sure you're installing the correct Ignition Gazebo packages for ROS 2 Humble, as Gazebo has been rebranded as Ignition Gazebo in this distribution.

## Troubleshooting Gazebo Issues

If you encounter issues with Gazebo simulation, particularly with the diff drive controller or missing libraries, make sure you have installed the required packages:

```bash
# Install diff drive controller
sudo apt install ros-humble-diff-drive-controller

# Install Gazebo packages
sudo apt install ros-humble-ros-ign-gazebo-demos
```

Also, make sure the URDF file is using the correct plugin names for Ignition Gazebo instead of the traditional Gazebo:

- Use `libignition-gazebo-diff-drive-system.so` instead of `libgazebo_ros_diff_drive.so`
- Update the plugin parameters to match Ignition Gazebo system requirements

## URDF Model Restoration

The URDF model in `simulation.urdf.xacro` was accidentally overwritten during updates. It has been restored with:

- A base link with proper visual, collision, and inertial properties
- Four wheels (front_left, front_right, rear_left, rear_right) with visual, collision, and inertial properties
- Proper joint definitions for all four wheels
- Correct Gazebo plugin configuration for differential drive control

## Robot Model Position Fix

Fixed the robot model positioning in Gazebo:
- Adjusted wheel positions so they are properly aligned with the ground
- Set base_link at the correct height to prevent wheels from being below ground level

## Troubleshooting Teleop Control Issues

If the robot is not responding to gamepad controls:

1. Make sure you're pressing Button 6 (the enable button) while using the joystick
2. Check that the teleop.launch.py file correctly loads the joystick.yaml configuration
3. Verify that the topic remapping is correct: `/cmd_vel` should be remapped to `/diff_drive_controller/cmd_vel_unstamped`
4. Confirm that the diff_drive_controller is properly loaded in the Gazebo simulation

## Gamepad Configuration Fix

Fixed the gamepad configuration loading issue:
- Updated the YAML configuration file with the correct node-specific parameter structure
- Modified the launch file to properly load parameters from the YAML file
- Ensured the enable button is correctly set to Button 6
- Directly set parameters in the launch file to ensure correct loading

## Direct Parameter Setting

Due to issues with parameter loading from YAML files, parameters are now directly set in the launch file to ensure the correct configuration:

- `enable_button` is set to 6 (Button 6 as the enable button)
- Linear axis x is mapped to Axis 1
- Angular yaw is mapped to Axis 0
- Proper scaling factors are applied to linear and angular velocities

## Gazebo Differential Drive Plugin Configuration

Fixed the Gazebo differential drive plugin configuration:
- Updated the wheel separation parameter to match the actual robot dimensions
- Verified the wheel radius parameter matches the physical wheel size
- Ensured all four wheels (front_left, front_right, rear_left, rear_right) are correctly configured in the plugin
- Updated the plugin to use the correct Ignition Gazebo diff drive system

## Topic Remapping Fix

Fixed the topic remapping issue that prevented Gazebo robot from responding to gamepad controls:
- Changed topic remapping from `/diff_drive_controller/cmd_vel_unstamped` to `/cmd_vel`
- This ensures the teleop node publishes directly to the Gazebo plugin's expected topic

## Yahboom Rosmaster Description Package Update

Updated the Yahboom Rosmaster description package to be compatible with ROS 2 Humble:

### 1. Controller Replacement

- Replaced the mecanum drive controller with the standard differential drive controller
- Updated configuration files to use `diff_drive_controller/DiffDriveController` instead of `mecanum_drive_controller/MecanumDriveController`
- Modified joint configurations to match differential drive requirements

### 2. Gazebo Plugin Update

- Updated the Gazebo plugin from mecanum drive to differential drive
- Changed plugin name from `gz::sim::systems::VelocityControl` to `gz::sim::systems::DiffDrive`
- Updated plugin parameters to match differential drive requirements

### 3. Configuration Files

- Modified `ros2_controllers.yaml` to use differential drive controller parameters
- Updated `ros2_controllers_template.yaml` with proper parameter templates
- Adjusted joint naming conventions to match differential drive controller expectations

### 4. Dependencies

- Added missing `urdf_tutorial` dependency to resolve build issues
- Ensured all required ROS 2 Humble packages are properly installed and configured

These changes make the Yahboom Rosmaster robot compatible with standard ROS 2 differential drive controllers, making it easier to integrate with existing ROS 2 navigation and control stacks.

## ISAAC ROS Bringup Package

Created a new `isaac_ros_bringup` package to integrate Yahboom Rosmaster robot with Gazebo simulation and joystick control. This package provides:

### 1. Features

- Launch Gazebo simulation with Yahboom Rosmaster robot
- Launch RViz2 for robot visualization
- Enable joystick control of the robot in both Gazebo and RViz2
- Pre-configured joystick mappings for common gamepads

### 2. Package Structure

- `launch/`: Contains launch files for simulation and control
- `config/`: Contains configuration files for joystick control

### 3. Launch Files

- `robot_bringup.launch.py`: Launches Gazebo simulation with robot and RViz2
- `joystick_control.launch.py`: Launches joystick control nodes

### 4. Dependencies

The package depends on:
- `yahboom_rosmaster_gazebo`: For Gazebo simulation support
- `yahboom_rosmaster_description`: For robot description
- `teleop_twist_joy`: For joystick teleoperation
- `joy`: For joystick input handling

### 5. Installation

To build the package:

```bash
cd /workspaces/isaac_ros-dev/isaac_ros_ws
colcon build --packages-select isaac_ros_bringup
source install/setup.bash
```

### 6. Usage

To launch the simulation and visualization:

```bash
ros2 launch isaac_ros_bringup robot_bringup.launch.py
```

To launch joystick control (in a separate terminal):

```bash
ros2 launch isaac_ros_bringup joystick_control.launch.py
```

### 7. Joystick Control

The default joystick configuration uses:
- Axis 1 (usually left stick vertical): Forward/backward movement
- Axis 0 (usually left stick horizontal): Left/right rotation
- Button 6 (usually LB/Left Bumper): Enable button (must be held while controlling)

This configuration works with most Xbox-style gamepads.