---
layout: default
title: Connecting to Robot
nav_order: 1
parent: Fetch
---

# Connecting to the Physical Fetch Robot

Follow these steps to connect to Fetch via wireless network.

1. **Network Configuration**:
   Ensure you are on the same network\Wifi connection as Fetch.

2. **Fetch Host IP**:
   In a terminal on Fetch's computer, get the host IP address by running: `ifconfig`

3. Look for the wireless device named something like `wlan0`. Take note of the `inet` address.

4. **Update Hosts File**:
   On your computer (not on Fetch), access your hosts file by entering: `sudo nano /etc/hosts`
   At the top of the file, you will see your local host aliases. Add a new alias with the inet address from the Fetch terminal output and the name of the Fetch computer: `<fetch-host-ip> fetch1102`

5. **Set ROS Master URI**:
   On your computer, set the ROS master URI by entering:
   `export ROS_MASTER_URI=http://fetch1102:11311`

6. **Run rviz**:
   You can now run rviz with:
   `rosrun rviz rviz`

7. Alternatively, run an RViz config if you have one set up:
   `rosrun rviz rviz -d /path/to/your_config_file.rviz`
