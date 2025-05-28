# ROS2 Project - Universal Robots UR5e
![title](/resources/title.png)
Control and Simulate Universal Robots `UR5e` Robot Platform with ROS2.

Prerequisite
---
- Ubuntu `Novel 24.04.x` LTS
- Linux Real-Time Kernel (PREEMPT_RT)
- ROS2 `Jazzy Jalisco` ([Official Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))
    ```shell
    $ sudo apt update && sudo apt install locales
    $ sudo locale-gen en_US en_US.UTF-8
    $ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    $ export LANG=en_US.UTF-8

    $ sudo apt install software-properties-common
    $ sudo add-apt-repository universe

    $ sudo apt update && sudo apt install curl -y
    $ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    $ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    $ sudo apt update && sudo apt upgrade -y
    $ sudo apt install -y ros-jazzy-desktop ros-dev-tools
    ```
    append below line to `~/.bashrc`
    ```
    source /opt/ros/jazzy/setup.sh
    ```
    set ros environment
    ```
    $ source ~/.bashrc
    ```
- Gazebo `Harmonic` ([Official Guide](https://gazebosim.org/docs/harmonic/ros_installation/))
    ```shell
    $ sudo apt install -y ros-${ROS_DISTRO}-ros-gz
    ```
- ROS2 Control ([Official Guide](https://control.ros.org/jazzy/doc/getting_started/getting_started.html#binary-packages))
    ```shell
    $ sudo apt install -y \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-gz-ros2-control
    ```
- MoveIt2 ([Official Guide](https://moveit.ai/install-moveit2/binary/))
    ```shell
    $ sudo apt install -y \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
    ```
    append below line to `~/.bashrc`
    ```
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
- Universal Robots ROS2 Driver ([Github Repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver))
    ```shell
    $ sudo apt-get install -y ros-${ROS_DISTRO}-ur
    ```
- Misc ROS Packages
    ```shell
    $ sudo apt install -y \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-rqt-joint-trajectory-controller
    ```

How to Use?
---
1. Clone repository
    ```shell
    $ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
    $ git clone https://github.com/YOGYUI/ros2_robot_ur5e.git
    ```
2. Build package
    ```shell
    $ cd ~/ros2_ws
    $ colcon build --packages-select ros2_robot_ur5e --symlink-install
    $ source ~/ros2_ws/install/local_setup.bash
    ```
3. Launch
    ```shell
    $ ros2 launch ros2_robot_ur5e robot.launch.py [real_hw:=true/false] [launch_gazebo:=true/false] [launch_rviz:=true/false]
    ```
    - Simulation (gazebo)
        ```shell
        $ ros2 launch ros2_robot_ur5e robot.launch.py real_hw:=false launch_gazebo:=true
        ```
        ![gazebo_simulation](/resources/gazebo_simulation.gif)
    - Real Hardware (UR5e)
        ```shell
        $ ros2 launch ros2_robot_ur5e robot.launch.py real_hw:=true
        ```
4. UR5e Configuration
    - Get calibration kinematics from robot
        ```shell
        $ ros2 launch ur_calibration calibration_correction.launch.py \
        robot_ip:=<robot_ip> \
        target_filename:="${HOME}/ros2_ws/src/ros2_robot_ur5e/config/ur_calibration.yaml"
        ```

Additional Setting for Real-Time Environment
---
Reference Link: [Setup for real-time scheduling](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/real_time.html)<br>
`Notes`: PREEMPT_RT Real-Time Linux Kernel should be installed first.
1. Install low latency kernel
    ```shell
    $ sudo apt install linux-lowlatency
    ```
2. Setup user privileges
    ```shell
    $ sudo groupadd realtime
    $ sudo usermod -aG realtime $(whoami)
    ```
    open `/etc/security/limits.conf` and add below lines.
    ```
    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400
    ```
