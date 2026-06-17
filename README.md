# hex_ros_sim_archer_y6

## What does this package do

This package demonstrates a structure of writing a python ROS package that can be used in **both ROS 1 and ROS 2**. 

It will wait for input, for string input, it will add a prefix to the input string and publish it to the `/out_str` topic. For int32 input, it will check if the input is within the specified range and publish it to the `/out_int` topic.

To use the demo, run a launch file in the `launch` directory, and publish to the `/in_str` and `/in_int` topics.

## Maintainer

[Dong Zhaorui](https://github.com/IBNBlank)

## Prerequisites

Ensure the following software and hardware are installed:

* **ROS**:  
   Refer to the [ROS Installation guide](http://wiki.ros.org/ROS/Installation)

### Verified Platforms

* [x] **x64**
* [ ] **Jetson Orin Nano**
* [x] **Jetson Orin NX**
* [ ] **Jetson AGX Orin**
* [ ] **Horizon RDK X5**
* [ ] **Rockchip RK3588**

## Public APIs

### Published Topics

| Topic      | Msg Type                | Description                      |
| ---------- | ----------------------- | -------------------------------- |
| `/out_str` | `std_msgs/(msg/)String` | Example of a string publication. |
| `/out_int` | `std_msgs/(msg/)Int32`  | Example of an int32 publication. |

### Subscribed Topics

| Topic     | Msg Type                | Description                       |
| --------- | ----------------------- | --------------------------------- |
| `/in_str` | `std_msgs/(msg/)String` | Example of a string subscription. |
| `/in_int` | `std_msgs/(msg/)Int32`  | Example of an int32 subscription. |

### Parameters

| Name        | Data Type     | Description                   |
| ----------- | ------------- | ----------------------------- |
| `str_name`  | `string`      | Prefix for the output string. |
| `int_range` | `vector<int>` | Range for the output int.     |

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

3. Clone the repository:

   ```shell
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

1. Launch the `py_template` node:

   For ROS 1:

   ```shell
   roslaunch hex_ros_sim_archer_y6 py_template.launch
   ```

   For ROS 2:

   ```shell
   ros2 launch hex_ros_sim_archer_y6 py_template.launch.py
   ```

2. Publish to `/in_str` and `/in_int` topics.
3. View the output on the `/out_str` and `/out_int` topics.
