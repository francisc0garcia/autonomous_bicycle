How to install Autonomous bicycle package
=========================================

In order to use this project, we must install first ROS (Robotic Operating System) (tested kinetic) on a Linux PC,
then compile the project and finally execute some simulations and tests.

Install ROS and dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow ROS installation procedure given on:

http://wiki.ros.org/kinetic/Installation/Ubuntu

we can summarize the steps:

Open a command windows on ubuntu and run the following commands:

- Prepare ubuntu for installation:

.. code-block:: none

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
    sudo apt-get update

- Install ROS

For PC/Laptop we should install full desktop version:

.. code-block:: none

    sudo apt-get install ros-kinetic-desktop-full

For Raspberry PI 3, Odroid or other embedded system, we can install ROS-Base:

.. code-block:: none

    sudo apt-get install ros-kinetic-ros-base

Install rosdep:

.. code-block:: none

    sudo rosdep init
    rosdep update

And finally prepare environment:

.. code-block:: none

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Get and Compile autonomous bicycle project
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First we need to install some dependencies and ROS packages:

.. code-block:: none

    sudo apt-get install libqwt-dev ros-kinetic-teleop-twist-joy  ros-kinetic-rviz-imu-plugin python-smbus ros-kinetic-rqt-multiplot git
    sudo apt-get install ros-kinetic-gps* ros-kinetic-jsk-rqt-plugins ros-kinetic-jsk-rviz-plugins ros-kinetic-rviz-imu-plugin
    sudo apt-get install ros-kinetic-hector-gazebo* ros-kinetic-mapviz*

Finally we create a workspace for project, clone github repositories, install dependencies and compile all:

.. code-block:: none

    mkdir -p ~/autonomous_bicycle_ws/src
    cd ~/autonomous_bicycle_ws/src
    git clone https://github.com/francisc0garcia/autonomous_bicycle
    git clone https://github.com/andreasBihlmaier/pysdf
    git clone https://github.com/gareth-cross/rviz_satellite
    cd ..
    catkin_make


If you get some error during compilation, check if all dependencies are installed using:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    rosdep install autonomous_bicycle
    catkin_make


Test project
^^^^^^^^^^^^

Once the project has been compiled successfully,
we can run a simulation that includes a bicycle and simple controller.

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle

if everything is correct, you should see a standing bicycle, which you can control using dynamic reconfigure plugin.
Now you are ready to play and extend the project, let's go to section Tutorials and extensions.