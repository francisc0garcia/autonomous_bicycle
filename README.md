# Autonomous bicycle

ROS package for localization, pose estimation and autonomous navigation algorithms of autonomous bikes.

Current available features:
- Simulation of bicycle in gazebo and synchronized with RVIZ (model + tf)
- RQT interface: control bicycle speed and plot lean and steering angles for stability analysis

Dynamic simulation of bicycle:

[![Autonomous bicycle](http://img.youtube.com/vi/t7ZZPeML2Fw/0.jpg)](https://www.youtube.com/watch?v=t7ZZPeML2Fw "Autonomous bicycle")

Sensor board on real bike:

[![Autonomous bicycle prototype](http://img.youtube.com/vi/9oQhyyu41Hg/0.jpg)](https://www.youtube.com/watch?v=9oQhyyu41Hg "Autonomous bicycle prototype")

## How to use 

Install dependencies:

    sudo apt-get install git ros-kinetic-hector-gazebo-plugins  ros-kinetic-rqt-multiplot

How to test it?

    cd path_ros_workspace/src/
    git clone https://github.com/francisc0garcia/autonomous_bicycle
    git clone https://github.com/gareth-cross/rviz_satellite
    git clone https://github.com/ccny-ros-pkg/imu_tools
    git clone https://github.com/andreasBihlmaier/pysdf
    cd .. 
    catkin_make
    source devel/setup.bash
    roslaunch autonomous_bicycle autonomous_bicycle.launch

Project under development, suggestions are welcome!

- Developed by:

        Francisco J. Garcia R.
        Alen Turnwald
        2017