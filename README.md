# hex_ros_sim_archer_y6

## What does this package do

This package provides a **MuJoCo simulation** of the Archer Y6 manipulator (6-DoF arm + gr100 gripper) that runs as a python ROS node usable in **both ROS 1 and ROS 2**.

The node:

* Steps a MuJoCo physics model at a configurable simulation rate and publishes the simulated time on `/clock`, so that other nodes can run with `use_sim_time`.
* Publishes the robot state (`HexRosRoboManipStateStamped`) at a configurable state rate.
* Optionally publishes `sensor_msgs/JointState` on `/joint_states` so that `robot_state_publisher` can compute TF transforms for RViz visualization.
* Subscribes to control commands (`HexRosRoboManipCtrlStamped`) and, on every simulation step, applies the **latest** received command.

An optional **test controller** node (`test_ctrl`) can be launched alongside the simulator. It cycles through preset arm and gripper control modes (MIT / JNT / EE) for quick validation.

Internally the ROS interface converts between the `hex_ros_msgs` ROS messages and the `hex_util_msg` dataclasses (`HexDcRoboManipStateStamped` / `HexDcRoboManipCtrlStamped`); the simulator is fully ROS-agnostic and only deals with those dataclasses.

## Maintainer

[Dong Zhaorui](https://github.com/IBNBlank)

## Prerequisites

Ensure the following software and hardware are installed:

* **ROS**:  
   Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)

### Verified Platforms

* [x] **x64**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**

## Public APIs

### Published Topics

| Topic          | Msg Type                              | Description                              |
| -------------- | ------------------------------------- | ---------------------------------------- |
| `/clock`       | `rosgraph_msgs/(msg/)Clock`           | Simulation time for `use_sim_time`.      |
| `/manip_state` | `hex_ros_msgs/(msg/)HexRosRoboManipStateStamped` | Simulated arm + gripper state. |
| `/joint_states`| `sensor_msgs/(msg/)JointState`        | Joint positions / velocities / efforts for TF & RViz. |

### Subscribed Topics

| Topic         | Msg Type                                        | Description              |
| ------------- | ----------------------------------------------- | ------------------------ |
| `/manip_ctrl` | `hex_ros_msgs/(msg/)HexRosRoboManipCtrlStamped` | Arm + gripper command.   |

### Parameters

| Name              | Data Type | Default        | Description                                                  |
| ----------------- | --------- | -------------- | ----------------------------------------------------------- |
| `rate_ros`        | `double`  | `1000.0`       | MuJoCo simulation step rate [hz] (sets the model timestep). |
| `rate_state`      | `double`  | `500.0`        | Robot state publish rate [hz].                              |
| `model_urdf`      | `string`  | `""`           | URDF path for dynamics computations (e.g. `HexDynUtilY6`).  |
| `model_mjcf`      | `string`  | `""`           | MuJoCo MJCF scene path.                                     |
| `model_frame_id`  | `string`  | `"base_link"`  | Frame id used in the published state header.                |
| `prog_viewer`     | `bool`    | `true`         | Enable the MuJoCo interactive viewer window.                |
| `prog_rviz`       | `bool`    | `true`         | Publish `/joint_states` for RViz / TF visualization.        |

The launch files expose `viewer`, `rviz`, and `test` as launch arguments that override `prog_viewer`, `prog_rviz`, and the test controller respectively.

## Getting Started

Follow these steps to set up the project for development and testing on your local machine:

1. Install necessary dependencies:

   ```shell
   pip3 install 'hex-util-msg>=0.1.0a0'
   pip3 install 'hex-util-ros>=0.0.1a0'
   ```

2. Create a workspace `catkin_ws` and navigate to the `src` directory:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   ```

3. Clone necessary repositories:

   ```shell
   git clone https://github.com/hexfellow/hex_ros_urdf_archer_y6.git
   git clone https://github.com/hexfellow/hex_ros_msgs.git
   git clone https://github.com/hexfellow/hex_ros_sim_archer_y6.git
   ```

4. Navigate back to the `catkin_ws` directory and build the workspace:

   For ROS 1:

   ```shell
   cd ../
   catkin_make
   ```

   For ROS 2:

   ```shell
   cd ../
   colcon build
   ```

5. Source the `setup.bash` file and run the tests:

   For ROS 1:

   ```shell
   source devel/setup.bash --extend
   ```

   For ROS 2:

   ```shell
   source install/setup.bash --extend
   ```

### Usage

1. Launch the `sim_archer_y6` node:

   For ROS 1:

   ```shell
   # bare simulation (no viewer, no rviz)
   roslaunch hex_ros_sim_archer_y6 sim_archer_y6.launch viewer:=false rviz:=false

   # with MuJoCo viewer and RViz visualization, plus the test controller
   roslaunch hex_ros_sim_archer_y6 sim_archer_y6.launch viewer:=true rviz:=true test:=true
   ```

   For ROS 2:

   ```shell
   # bare simulation (no viewer, no rviz)
   ros2 launch hex_ros_sim_archer_y6 sim_archer_y6.launch.py viewer:=false rviz:=false

   # with MuJoCo viewer and RViz visualization, plus the test controller
   ros2 launch hex_ros_sim_archer_y6 sim_archer_y6.launch.py viewer:=true rviz:=true test:=true
   ```

2. Subscribe to `/manip_state` to read the simulated robot state.
3. Publish `HexRosRoboManipCtrlStamped` to `/manip_ctrl` to control the robot.
4. (Optional) Start `test_ctrl` with `test:=true` to run the preset control-mode cycle.
5. Start any downstream node with `use_sim_time` enabled so it follows the `/clock` published by the simulation.
