---
layout: default
title: ROS Galactic Setup
nav_order: 1
parent: LoCoBot
---

# ROS Galactic Setup

Currently, only the **LoCoBot** platform is running a ROS 2 version. If you are using the Fetch platform, please refer to the [ROS 1 Melodic Installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu)
{: .note }

## Installation

The following ROS installation steps are from the [Official Galactic Installation Guide.](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html) Refer there for more detail.
{: .highlight }

Verify locale settings (mainly for minimal environments like a Docker container).

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

Ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update your apt repository caches after setting up the repositories.

```bash
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

```bash
sudo apt upgrade
```

Install ROS 2 packages.

```bash
sudo apt install ros-galactic-desktop
```

## Bash Configuration

Open your `.bashrc` file with a text editor.

```bash
cd
nano ~/.bashrc
```

Add the following lines to the bottom of the file.

```bash
alias galactic='source /opt/ros/galactic/setup.bash'
export INTERBOTIX_XSLOCOBOT_BASE_TYPE=create3
```

Restart your `.bashrc` to reflect these changes.

```bash
source ~/.bashrc
```

Now you can source your ROS environment in a new terminal by simply typing.

```bash
galactic
```

## Installing Additional Dependencies

Install and initialize Rosdep

```bash
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

Now we will install Colcon (the ROS 2 build tool).

```bash
sudo apt install python3-colcon-common-extensions
```

And finally, we will initialize our Colcon workspace.

```bash
mkdir -p ~/colcon_ws/src
cd colcon_ws
colcon build
```
