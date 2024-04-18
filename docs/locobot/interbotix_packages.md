---
layout: default
title: Interbotix Packages
nav_order: 3
parent: LoCoBot
---

# Installing the Interbotix ROS Package Suite

This guide covers the steps necessary to install and build the `interbotix_ros_rovers` package to start developing on the LoCoBot.

This guide is made for **ROS 2 Galactic** running on **Ubuntu 20.04 (Focal)**.
{: .note }

## Installing `interbotix_ros_rovers` Package Dependencies

The four main Interbotix packages are dependent on eachother in the following way:

```
# Tree is ONLY a way to show dependencies, all these packages will go in '~/colcon_ws/src'
interbotix_ros_rovers
  └── interbotix_ros_toolboxes
  └── interbotix_ros_core
      └── interbotix_ros_xs_driver
```

Install `interbotix_ros_toolboxes` package in your `~/colcon_ws/src` directory.

```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_toolboxes.git
```

Install the required C++ YAML parser.

```bash
sudo apt-get install libyaml-cpp-dev
```

Install the `interbotix_ros_core` package.

```bash
git clone https://github.com/Interbotix/interbotix_ros_core.git -b galactic
```

Initialize the `dynamixel_workbench_toolbox` package.

```bash
cd interbotix_ros_core
git submodule init interbotix_ros_xseries/dynamixel_workbench_toolbox/
git submodule update
rm interbotix_ros_xseries/COLCON_IGNORE
```

Install Missing dependencies (these were the ones I needed to install, it may be different for other systems).

```bash
sudo apt install ros-galactic-dynamixel-sdk ros-galactic-gazebo-ros-pkgs ros-galactic-tf-transformations ros-galactic-ros2-control ros-galactic-ros2-controllers
```

Add the `interbotix_xs_driver` drivers package to your `~/colcon_ws/src`.

```bash
git clone https://github.com/Interbotix/interbotix_xs_driver.git
```

Now build your workspace from `~/colcon_ws` (the root of your workspace) to make sure everything is working.

```bash
galactic
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro=galactic
colcon build
```

If the build was successful, it is time to install all the LoCoBot packages. Clone the following repo into your `~/colcon_ws/src` directory.

```bash
git clone -b galactic https://github.com/Interbotix/interbotix_ros_rovers.git
```

And finally, try to build the workspace once more.

```bash
cd ..
colcon build
```

If you are getting errors regarding missing paths, try deleting the `install` and `build` directories from your workspace and rebuilding.

```bash
rm -rf install build
colcon build
```

## Installing Even More Dependencies

Since our LoCoBot is using a Create3 base, we're going to need some packages from the `create3_sim` repo to simulate it in Gazebo. Clone the following repository **somewhere other than your Colcon workspace directory (for example, your desktop)**.

```bash
git clone -b galactic https://github.com/iRobotEducation/create3_sim.git
```

From this new directory, drag the `irobot_create_common` and `irobot_create_gazebo` packages into your `~/colcon_ws/src` directory.

Now, staying in your `~/colcon_ws/src` directory, clone the following packages.

```bash
git clone -b galactic https://github.com/iRobotEducation/irobot_create_msgs.git
git clone https://github.com/Slamtec/sllidar_ros2.git
```

Now we will install some missing dependencies from the ROS package manager. Similar to before, these were the ones I had to install, I'm not sure if it will be the same for every device. These also don't seem to be picked up by `rosdep update` for whatever reason.

```bash
sudo apt install ros-galactic-gazebo-ros2-control ros-galactic-rtabmap-ros ros-galactic-joint-state-publisher ros-galactic-joint-state-publisher-gui ros-galactic-nav2-bringup ros-galactic-rplidar-ros ros-galactic-realsense2-camera ros-galactic-moveit
```

Now run Rosdep to check for more missing dependencies. If it says there are missing dependencies, try installing them from the ROS package manager (like above) or searching for them on GitHub. Run the following command from `~/colcon_ws` not `~/colcon_ws/src`

```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro=galactic
```

If there are no missing dependencies, try to build your workspace.

```bash
colcon build
```

If you are getting errors regarding missing paths, try deleting the `install` and `build` directories from your workspace and rebuilding.

```bash
rm -rf install build
colcon build
```

## Launching Gazebo Simulation

First, we need to make sure that the Gazebo environment is being ran on every new terminal window. Open your `.bashrc` file with a text editor.

```bash
cd
nano ~/.bashrc
```

Add the following line to the bottom of the file.

```bash
. /usr/share/gazebo/setup.sh
```

Restart your `.bashrc` to reflect these changes.

```bash
source ~/.bashrc
```

Now you can either launch Gazebo and Moveit simultaneously using this command:

```bash
ros2 launch interbotix_xslocobot_moveit xslocobot_moveit.launch.py robot_model:=locobot_wx200 use_lidar:=true hardware_type:=gz_classic
```

Or you can just run Gazebo without a Moveit instance.

```bash
ros2 launch interbotix_xslocobot_sim xslocobot_gz_classic.launch.py robot_model:=locobot_wx200
```

I've found that as a general rule, if you're having trouble with Gazebo, the first thing you should try is to kill all client and server instances. If Gazebo is not opening for you, try running the following command.

```bash
killall gzserver gzclient
```
