# ROS2 Project - Universal Robots UR5e
ROS2 Project - Control and Simulate **Universal Robotics UR5e** Robot Platform

Prerequisite
---
- Ubuntu `Novel 24.04.x` LTS
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
$ ros2 launch ros2_robot_ur5e robot.launch.py
```