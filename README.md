# Rover Simulation 


## Setup
1. Clone the repository:
```bash
git clone https://github.com/nickoenig37/rover_simulation.git
```

Getting started with updating and fetching Submodules:
```bash
git submodule update --init --recursive --remote
```

## Install Docker on your system (IF YOU DON'T HAVE IT ALREADY)
```bash
cd ORB-SLAM3-ROS2-MONO-Docker
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```


# Notes from the OG repo:
# 3. Build the image with ORB_SLAM3

1. Build the image: ```sudo docker build --build-arg USE_CI=false -t orb-slam3-humble:22.04 .```
2. Add ```xhost +``` to your ```.bashrc``` to support correct x11-forwarding using ```echo "xhost +" >> ~/.bashrc```
3. ```source ~/.bashrc```
4. You can see the built images on your machine by running ```sudo docker images```.

## 4. Running the container

1. ```cd ORB-SLAM3-ROS2-Docker``` (ignore if you are already in the folder)
2. ```sudo docker compose run orb_slam3_22_humble```
3. This should take you inside the container. Once you are inside, run the command ```xeyes``` and a pair of eyes should pop-up. If they do, x11 forwarding has correctly been setup on your computer.

## 5. Building the ORB-SLAM3 Wrapper

Launch the container using steps in (4).
```bash
cd /home/orb/ORB_SLAM3/ && sudo chmod +x build.sh && ./build.sh
cd /root/colcon_ws/ && colcon build --symlink-install && source install/setup.bash
```

## Launching ORB-SLAM3

Launch the container using steps in (4).
If you are inside the container, run the following:

1. ```ros2 launch orb_slam3_ros2_wrapper unirobot.launch.py```
3. You can adjust the robot namespace in the ```unirobot.launch.py``` file.

## Running this with a Gazebo Sim simulation.

1. clone the repo for the simulation environment:
```bash
git clone -b humble https://github.com/suchetanrs/gz-sim-environment.git
```
2. Follow: 
```bash
cd gz-sim-environment
sudo chmod +x install-nvidia-container-toolkit.sh
./install-nvidia-container-toolkit.sh
```
3. Pull the latest image:
```bash
sudo docker pull nvidia/cuda:11.4.2-cudnn8-runtime-ubuntu20.04
sudo docker build -t gazebo-vehicle-ros2-gpu-harmonic:humble .
```
4. Run this:
```bash
sudo docker compose run vehicle_simulator_gz_sim
```
8. ```sudo chmod +x launch_simulation.sh```
9. ```./launch_simulation.sh```
10. This will build the packages in the top-right terminal.
11. After the build is complete you can run what was inputted in the top-left terminal. This will launch gazebo and spawn the robot.
12. The GUI is disabled by default.
13. You should be able to teleop the robot through the teleop window.
14. You can run the default simulation RViz from the bottom-right terminal. As long as you are able to see the lidar pointcloud, the rgb and depth images on RViz, you can safely ignore the texture error messages on the simulation terminal window.
15. If you wish to launch cave world, you can do run the following in the top right terminal: `ros2 launch vehicle_bringup unirobot.launch.py world:=cave_world.sdf`
16. If you wish to launch the corridor world, you can do run the following in the top right terminal: `ros2 launch vehicle_bringup unirobot.launch.py world:=indoor.sdf`

**Once you are able to teleop the robot, you should be able to run ORB-SLAM3 with both the containers (simulation and wrapper) running parallely.**

### Important information
The ```ROS_DOMAIN_ID``` is set to 55 in the ```.bashrc```

There are currently two model types. More will be available soon! (Drones, Legged Robots, Multi-Robot Simulations)

You can modify the ```spawn_robot.launch.py``` in the ```vehicle_packages``` to select your desired model.

Edit the `robot_model_type` variable.

## Running the map_generator package.

This package can be used to generate a global pointcloud from the SLAM.
It subscribes to the published `map_data` and an input pointcloud either from a LiDAR or from a depth camera. It stitches the input pointclouds together based on the latest pose-graph data.

To run the package, these steps can be followed:
1. Make sure the simulation is running. Run the orb_slam3 container and once you are in the bash shell, run the following: `./launch_slam.sh`
2. The top-left terminal contains the launch file to run the slam. This must be launched first.
3. The bottom-left terminal contains the launch file to start the pointcloud_stitcher node. This should run soon after you launch the SLAM.
4. If you wish to publish the global pointcloud at any point during the SLAM's operation, simply run the python file in the top-right terminal. You should be able to view the global pointcloud in rviz (you can launch RViz with the correct configuration from the bottom-right terminal).

### Potential issues you may face.
The simulation and the wrapper both have their ```ROS_DOMAIN_ID``` set to 55 so they are meant to work out of the box. However, you may face issues if this environment variable is not set properly. Before you start the wrapper, run ```ros2 topic list``` and make sure the topics `/rgb_camera` and `/depth_camera` are visible inside the ORB-SLAM3 container provided the simulation is running along the side.

## Services
| Service Name          | Purpose | type |
|-------------------------|---------------|---------------|
| `orb_slam3/get_map_data`      | Sends the map_data in the response. | `slam_msgs::srv::GetMap` |
| `orb_slam3/get_landmarks_in_view`      | Takes an input pose and publishes the feature points visible from that pose. | `slam_msgs::srv::GetLandmarksInView` |
| `orb_slam3/get_all_landmarks_in_map`      | Publishes all feature points in the map and fills the same pointcloud in the response. | `slam_msgs::srv::GetAllLandmarksInMap` |
| `orb_slam3/reset_mapping`      | Resets the current mapping instance and clears all keyframes. | `std_srvs::srv::SetBool` |

## Published Topics
| Topic Name          | Purpose | type |
|-------------------------|---------------|---------------|
| `map_points`            | Publishes the point cloud representing feature points collected from the SLAM process. This is published when `orb_slam3/get_all_landmarks_in_map` service is called.             | `sensor_msgs::msg::PointCloud2` |
| `visible_landmarks`     | Publishes the point cloud of feature points (landmarks) visible from a given pose. This is published when `orb_slam3/get_landmarks_in_view` service is called.                  | `sensor_msgs::msg::PointCloud2` |
| `slam_info`             | Publishes overall SLAM-related information.          | `slam_msgs::msg::SlamInfo`      |
| `map_data`              | Continuously publishes the map data generated by the SLAM algorithm.                               | `slam_msgs::msg::MapData`       |
| `robot_pose_slam`       | Publishes the robot's pose expressed in the global frame.                                        | `geometry_msgs::msg::PoseStamped` |

## ROS Parameters
| Parameter Name          | Default Value | Description                                                                 |
|-------------------------|---------------|-----------------------------------------------------------------------------|
| `robot_base_frame`      | `base_footprint` | The name of the frame attached to the robot's base. |
| `global_frame`          | `map`         | The name of the global frame of reference. It represents a fixed world coordinate frame in which the robot navigates.|
| `odom_frame`            | `odom`        | The name of the odometry frame. |
| `rgb_image_topic_name` | `rgb_camera` | The topic to recieve rgb images. |
| `depth_image_topic_name` | `depth_camera` | The topic to recieve depth images. |
| `imu_topic_name` | `imu` | The topic to recieve IMU messages (Not used in RGB-D mode). |
| `visualization`         | `true`        | A boolean flag to enable or disable visualization. When set to `true`, the ORB-SLAM3 viewer will show up with the tracked points and the keyframe trajectories.|
| `odometry_mode`      | `false`       | A boolean flag to toggle odometry mode. When `false`, the system operates without relying on odometry data, which might be used in scenarios where odometry information is unavailable or unreliable. In this case, it publishes the transform directly between the ```global_frame``` and the ```robot_base_frame```. Further information can be found on the [FAQ](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/wiki/FAQs)|
| `publish_tf`               | `true`         | Publishes the map->odom tf in case `odometry_mode` is set to `true` and map->odom->base_link in case odometry_mode is set to `false`. Further information can be found on the [FAQ](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/wiki/FAQs)|
| `map_data_publish_frequency`| `1000`         | Time interval at which map_data should be published (ms).|
| `do_loop_closing`| `true`         | Enable or disable loop closing in ORB-SLAM3. This will also disable re-localisation and multi-map if `false`|

## Important notes

ORB-SLAM3 is launched from ```orb_slam3_docker_20_humble/orb_slam3_ros2_wrapper/launch/rgbd.launch.py``` which inturn is launched from ```orb_slam3_docker_20_humble/orb_slam3_ros2_wrapper/launch/unirobot.launch.py```

Currently the ```rgbd.launch.py``` launch file defaults to ```orb_slam3_ros2_wrapper/params/gazebo_rgbd.yaml```. You can modify this with your own parameter file in case you wish to use your own camera.

The very initial versions of this code were derived from [thien94/orb_slam3_ros_wrapper](https://github.com/thien94/orb_slam3_ros_wrapper) and [zang9/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2)