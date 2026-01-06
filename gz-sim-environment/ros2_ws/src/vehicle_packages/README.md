# Vehicle Simulation - Quick Start Guide

## Prerequisites
- Container with NVIDIA GPU support
- ROS 2 Humble
- Gazebo Sim 8

## Building the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Option 1: Launch Simulation with GUI (Recommended)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch vehicle_bringup unirobot.launch.py
```

### Option 2: Using the Tmux Script
```bash
cd ~
./launch_simulation.sh
```
This creates a 4-pane tmux session with:
- **Pane 0**: Gazebo simulation
- **Pane 1**: Octomap server
- **Pane 2**: Build workspace
- **Pane 3**: RViz visualization

Press `Ctrl+B` then `D` to detach from tmux.

## Launch Options

### Different Worlds
```bash
# Cave world
ros2 launch vehicle_bringup unirobot.launch.py world:=cave_world.sdf

# Indoor corridor
ros2 launch vehicle_bringup unirobot.launch.py world:=indoor.sdf

# Mars yard
ros2 launch vehicle_bringup unirobot.launch.py world:=marsyard2020_walls.sdf
```

### Headless Mode (No GUI)
```bash
ros2 launch vehicle_bringup unirobot.launch.py headless:=true
```

## Viewing Sensor Data

### Launch RViz
```bash
rviz2 -d ~/simulation.rviz
```

### Check Available Topics
```bash
ros2 topic list
```

Key topics:
- `/lidar/points` - 3D LiDAR point cloud
- `/rgb_camera` - RGB camera image
- `/depth_camera` - Depth camera image
- `/odom_differential` - Wheel odometry
- `/ground_truth_pose` - Ground truth position

### Teleoperate the Robot
Use the xterm window that opens automatically, or:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_teleop
```

## Robot Models

Edit `spawn_robot.launch.py` to change the robot model:
```python
robot_model_type = "small_vehicle"
```

Available models:
- `small_vehicle` - Compact robot with camera and 3D LiDAR
- `small_vehicle_vert_lidar` - Compact with vertical LiDAR
- `small_vehicle_2d_lidar` - Compact with 2D LiDAR
- `model` - Full-size robot
- `model_with_2_lidar` - Full-size with dual LiDAR

## Troubleshooting

**Simulation won't start:**
- Ensure workspace is built: `colcon build --symlink-install`
- Source setup: `source ~/ros2_ws/install/setup.bash`

**No GUI appears:**
- Check DISPLAY: `echo $DISPLAY`
- Verify X server: `xdpyinfo | head`

**No sensor data in RViz:**
- Ensure simulation is running: `ros2 topic list`
- Check topic names match in RViz configuration
