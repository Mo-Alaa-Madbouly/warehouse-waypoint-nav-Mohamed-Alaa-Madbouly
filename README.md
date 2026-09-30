# Warehouse Waypoint Navigation

Autonomous navigation of a custom differential-drive robot (**`two_wheel_robot`**) inside the ETGAH warehouse world, built with **ROS 2 Jazzy**, **Gazebo Harmonic**, **SLAM Toolbox** and **Nav2**.

**Author:** Mohamed Alaa Madbouly


---

## Table of Contents

1. [Project Overview and Mission](#1-project-overview-and-mission)
2. [Repository and Package Structure](#2-repository-and-package-structure)
3. [Workspace Build Instructions](#3-workspace-build-instructions)
4. [Launching the Robot Inside the Warehouse World](#4-launching-the-robot-inside-the-warehouse-world)
5. [Mapping the Warehouse with SLAM Toolbox](#5-mapping-the-warehouse-with-slam-toolbox)
6. [Saving the Warehouse Map](#6-saving-the-warehouse-map)
7. [Launching and Testing AMCL Localization](#7-launching-and-testing-amcl-localization)
8. [Launching the Complete Nav2 System](#8-launching-the-complete-nav2-system)
9. [Mission Route](#9-mission-route)
10. [Screenshots](#10-screenshots)
11. [Videos](#11-videos)

---

## 1. Project Overview and Mission

This project takes a custom-built robot from description to autonomous navigation:

1. **Robot description** – a differential-drive robot (`two_wheel_robot`) modelled in URDF/xacro with an RPLIDAR-style 2D lidar and a ZED camera, simulated in Gazebo Harmonic.
2. **Mapping** – SLAM Toolbox builds a 2D occupancy map of the ETGAH warehouse world.
3. **Localization** – AMCL localizes the robot on the saved map.
4. **Navigation** – the Nav2 stack plans and drives the robot to goal poses inside the warehouse.

**Mission:** navigate the robot autonomously through the warehouse, visiting the mission waypoints in order without collisions (see [Mission Route](#9-mission-route)).

|Component|Version / Choice|
|---|---|
|ROS 2|Jazzy|
|Simulator|Gazebo Harmonic (`gz-sim`) via `ros_gz_sim` / `ros_gz_bridge`|
|Robot|`two_wheel_robot` (custom, package `robot_description`)|
|World|ETGAH warehouse world (`worlds/warehouse_storage.sdf`)|
|Mapping|SLAM Toolbox (online async)|
|Localization|Nav2 AMCL|
|Navigation|Nav2 (NavFn global planner, DWB local controller)|

**Robot summary**

- Differential drive: wheel separation 0.27 m, wheel radius 0.06 m, plus a fixed caster.
- Frames: `odom` → `base_footprint` → `base_link` → `lidar_link`, `zed_camera_link`, wheels.
- 2D lidar on `/scan` (30 Hz) and camera on `/camera/image_raw` (10 Hz).

---

## 2. Repository and Package Structure

```
warehouse-waypoint-nav-Mohamed-Alaa-Madbouly/
├── README.md
└── src/
    ├── robot_description/
    │   ├── config/gz_bridge.yaml
    │   ├── launch/
    │   │   ├── display.launch.py
    │   │   └── gazebo.launch.py
    │   ├── meshes/                 (lidar.STL, zed.STL)
    │   ├── models/                 (Depot, etgah_logo)
    │   ├── urdf/
    │   │   ├── robot_description.urdf.xacro
    │   │   └── robot_description.gazebo
    │   ├── worlds/warehouse_storage.sdf
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── slam_toolbox_demo/
    │   ├── config/
    │   │   ├── slam_toolbox_online_async.yaml
    │   │   └── slam_toolbox_localization.yaml
    │   ├── launch/
    │   │   ├── slam_toolbox_online_async.launch.py
    │   │   └── localization.launch.py
    │   ├── map/warehouse_world_map.{pgm,yaml}
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── robot_navigation/
        ├── config/
        │   ├── amcl.yaml
        │   ├── planner_server.yaml
        │   ├── controller_server.yaml
        │   ├── behavior_server.yaml
        │   └── bt_navigator.yaml
        ├── launch/
        │   ├── amcl.launch.py
        │   └── nav2_bringup.launch.py
        ├── map/warehouse_world_map.{pgm,yaml}
        ├── scripts/send_goal.py
        ├── CMakeLists.txt
        └── package.xml
```

|Package|Purpose|
|---|---|
|`robot_description`|Robot model (URDF/xacro), Gazebo plugins, ROS↔Gazebo bridge, warehouse world and models, Gazebo launch file.|
|`slam_toolbox_demo`|SLAM Toolbox configuration and launch files for mapping and localization mode, plus the saved map.|
|`robot_navigation`|AMCL and Nav2 configuration, launch files, the saved map and the goal-sending script.|

---

## 3. Workspace Build Instructions

The repository root is the workspace (it contains `src/`).

```bash
# 1. Clone the repository as your workspace
git clone https://github.com/Mo-Alaa-Madbouly/warehouse-waypoint-nav-Mohamed-Alaa-Madbouly.git ~/warehouse_ws
cd ~/warehouse_ws

# 2. Install ROS 2 Jazzy dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-teleop-twist-keyboard

# 3. Build
colcon build --symlink-install

# 4. Source the workspace (do this in every new terminal)
source /opt/ros/jazzy/setup.bash
source ~/warehouse_ws/install/setup.bash
```

---

## 4. Launching the Robot Inside the Warehouse World

```bash
ros2 launch robot_description gazebo.launch.py
```

This launch file:

- sets `GZ_SIM_RESOURCE_PATH` so the world can find the `Depot` and `etgah_logo` models,
- starts Gazebo Harmonic with `worlds/warehouse_storage.sdf` (server only, headless rendering),
- publishes the robot description with `robot_state_publisher`,
- spawns `two_wheel_robot` at `(0, 0, 0)`,
- starts `ros_gz_bridge` using `config/gz_bridge.yaml`.

Gazebo runs as a server without a window. To watch the simulation, open the GUI in another terminal:

```bash
gz sim -g
```

**Bridged topics**

|ROS topic|Type|Direction|
|---|---|---|
|`/clock`|`rosgraph_msgs/Clock`|Gazebo → ROS|
|`/cmd_vel`|`geometry_msgs/Twist`|ROS → Gazebo|
|`/odom`|`nav_msgs/Odometry`|Gazebo → ROS|
|`/tf`|`tf2_msgs/TFMessage`|Gazebo → ROS|
|`/joint_states`|`sensor_msgs/JointState`|Gazebo → ROS|
|`/scan`|`sensor_msgs/LaserScan`|Gazebo → ROS|
|`/camera/image_raw`, `/camera/camera_info`|`sensor_msgs/Image`, `CameraInfo`|Gazebo → ROS|

Quick check:

```bash
ros2 topic list
ros2 topic echo /scan --once
```

To inspect only the robot model in RViz (no simulation): `ros2 launch robot_description display.launch.py`.

---

## 5. Mapping the Warehouse with SLAM Toolbox

Open each command in its own terminal (with the workspace sourced).

```bash
# Terminal 1 – simulation
ros2 launch robot_description gazebo.launch.py

# Terminal 2 – SLAM Toolbox (online async mapping)
ros2 launch slam_toolbox_demo slam_toolbox_online_async.launch.py

# Terminal 3 – drive the robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4 – RViz to watch the map grow
rviz2 --ros-args -p use_sim_time:=true
```

In RViz set the **Fixed Frame** to `map`, then add **Map** (`/map`), **LaserScan** (`/scan`) and **RobotModel** displays.

Key settings in `slam_toolbox_online_async.yaml`:

|Parameter|Value|
|---|---|
|`mode`|`mapping`|
|`scan_topic`|`/scan`|
|`odom_frame` / `map_frame` / `base_frame`|`odom` / `map` / `base_footprint`|
|`resolution`|0.05 m|
|`max_laser_range`|12.0 m|
|`do_loop_closing`|`true`|

Drive slowly through every aisle and revisit areas so loop closure can correct drift. The launch file configures and activates the SLAM Toolbox lifecycle node automatically.

---

## 6. Saving the Warehouse Map

When the map is complete, save it:

```bash
cd ~/warehouse_ws/src/robot_navigation/map
ros2 run nav2_map_server map_saver_cli -f warehouse_world_map
```

This creates:

- `warehouse_world_map.pgm` – occupancy image (604 × 302 px)
- `warehouse_world_map.yaml` – metadata: resolution 0.05 m/px, origin `[-7.942, -6.593, 0]`

The map is stored in both `robot_navigation/map/` (used by AMCL and Nav2) and `slam_toolbox_demo/map/`. Rebuild after saving so the installed copy is updated:

```bash
cd ~/warehouse_ws && colcon build --symlink-install
```

---

## 7. Launching and Testing AMCL Localization

```bash
# Terminal 1 – simulation
ros2 launch robot_description gazebo.launch.py

# Terminal 2 – map server + AMCL
ros2 launch robot_navigation amcl.launch.py

# Terminal 3 – RViz
rviz2 --ros-args -p use_sim_time:=true
```

`amcl.launch.py` starts `map_server`, `amcl` and a lifecycle manager that activates both.

Main AMCL settings (`amcl.yaml`): differential motion model, likelihood-field laser model, 500–2000 particles, `scan_topic: scan`, `base_frame_id: base_footprint`, and the initial pose set to `(0, 0, 0)` (the robot's spawn pose).

**Testing localization**

1. In RViz set the Fixed Frame to `map` and add **Map** (`/map`, durability _Transient Local_), **LaserScan** (`/scan`) and **PoseArray** (`/particle_cloud`).
    
2. If needed, click **2D Pose Estimate** and set the robot's true pose.
    
3. Drive with `ros2 run teleop_twist_keyboard teleop_twist_keyboard` and check that the particle cloud converges around the robot.
    
4. Check that the `map → odom` transform is published:
    
    ```bash
    ros2 run tf2_ros tf2_echo map odom
    ```
    

Localization is good when the laser scan stays aligned with the map walls as the robot moves.

---

## 8. Launching the Complete Nav2 System

```bash
# Terminal 1 – simulation
ros2 launch robot_description gazebo.launch.py

# Terminal 2 – map server, AMCL and the Nav2 servers
ros2 launch robot_navigation nav2_bringup.launch.py

# Terminal 3 – RViz
rviz2 --ros-args -p use_sim_time:=true
```

Set the initial pose in RViz if the robot is not at the spawn point, then send a goal with **Nav2 Goal** in RViz, or with the goal script:

```bash
# Terminal 4 – sends a NavigateToPose goal to the running Nav2 stack
ros2 run robot_navigation send_goal.py
```

`send_goal.py` sends one `NavigateToPose` goal in the `map` frame (edit `x`, `y`, `yaw` in `main()`), prints the remaining distance as the robot drives, and reports when navigation finishes.

**What `nav2_bringup.launch.py` starts**

|Node|Config file|Role|
|---|---|---|
|`map_server`|`map/warehouse_world_map.yaml`|Serves the saved map|
|`amcl`|`amcl.yaml`|Localization|
|`planner_server`|`planner_server.yaml`|Global planner (`NavfnPlanner`) and global costmap|
|`controller_server`|`controller_server.yaml`|Local controller (`DWBLocalPlanner`) and local costmap|
|`behavior_server`|`behavior_server.yaml`|Recovery behaviors: spin, back-up, drive-on-heading, wait, assisted teleop|
|`bt_navigator`|`bt_navigator.yaml`|Behavior-tree navigator (`navigate_to_pose`, `navigate_through_poses`)|
|`lifecycle_manager_navigation`|–|Brings up all of the above in order|

Costmaps: the global costmap uses static, obstacle, denoise and inflation layers (inflation radius 0.5 m); the local costmap is a rolling window in the `odom` frame. Goal tolerances are 0.25 m (position) and 0.25 rad (yaw).

---

## 9. Mission Route

```
TODO: Start (0, 0) → Waypoint 1 → Waypoint 2 → Waypoint 3 → Waypoint 4
```

`TODO`: one or two sentences on how the route is executed (each waypoint sent as a `NavigateToPose` goal, next waypoint sent when the previous one succeeds).

---

## 10. Screenshots

Place the images in an `images/` folder at the repository root and update the file names below.

| Stage                                 | Screenshot                                                       |
| ------------------------------------- | ---------------------------------------------------------------- |
| Robot in the warehouse world (Gazebo) | ![gazebo](<attatchments/Pasted image 20260917191400.png>)        |
| Mapping with SLAM Toolbox             | ![mapping](<attatchments/vlcsnap-2026-09-28-20h13m06s432.png>)   |
| Saved warehouse map                   | ![map](<attatchments/Pasted image 20260928202953.png>)           |
| AMCL localization (particle cloud)    | ![amcl](<attatchments/Pasted image 20260928203215.png>)          |
| Navigation in progress                | ![nav](<attatchments/vlcsnap-2026-09-28-20h15m04s589 1.png>)     |

---

## 11. Videos

mapping
1
![mapping video 1](<attatchments/map1.mp4>)

2
![mapping video 2](attatchments/map2.mp4)

navigation
![navigation video](attatchments/nav.mp4)
