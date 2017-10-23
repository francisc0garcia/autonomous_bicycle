Tutorials related with configuration
====================================

This set of tutorials covers some configuration procedures that are useful for working with the developed system.
This tutorial assumes that the autonomous_bicycle project was successfully installed, compiled and tested according to the installation guide.

How to configure GPS receivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default, both GPS receivers work at *1 HZ* and a serial baud rate of *9600 bps*, but this is too slow for our application,
it is possible to increase this setup using the a provided launch file.

Please check before that both GPSs are available on ports *GPS_front_port* and *GPS_rear_port* given on the launch file.

.. code-block:: none

    cd ~/Documents/code/autonomous_bicycle
    source devel/setup.bash
    roslaunch autonomous_bicycle config_GPS_sensors.launch

If the process ends correctly, both GPS will work at *5 Hz* and a serial baud rate of *115200 bps*

*Important*: The measurement system assumes that both GPS are properly configured,
if you get a red color for GPS data on the web interface, it might be caused by misconfiguration of the receivers.
Please run again the configuration steps.

How to enable/disable service for auto-starting
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is possible to run the init ROS launch file at boot time, using ubuntu services.

To create a service that executes automatically the launch file for initialization, we are going to use *robot_upstart* package available on:

.. code-block:: none

    http://wiki.ros.org/robot_upstart

The first step is to install *robot_upstart* (in case it is not already installed) by typing:

.. code-block:: none

    sudo apt-get install ros-kinetic-robot-upstart

Then, create an empty package with a launch file that calls the initialization launch file of the bicycle.
It is possible to copy an existing *robot_starter* package.
Once the package is installed, run the following command line for creating and installing the ubuntu service:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    rosrun robot_upstart install robot_starter/launch/robot_starter.launch

Please notice that you also should run the command returned as output from the *robot_upstart* installation.

Following with the process, modify the *ROS_IP* and *ROS_MASTER_URI* variables for the new service by typing:

.. code-block:: none

    sudo nano /usr/sbin/robot-start

Add or change the following lines:

.. code-block:: none

    export ROS_IP=192.168.1.201
    export ROS_MASTER_URI=127.0.0.1

And comment out (using *#*) the line that defines the *ROS_HOSTNAME*

.. code-block:: none

    # export ROS_HOSTNAME=localhost

Finally, you can start or stop the service using:

.. code-block:: none

    sudo service robot start
    sudo service robot stop

The new service will be executed automatically on the next boot.

**Important**: The service runs the launch file using a different user, it is necessary to grant rights
if you are going to use nodes that requires root access. Please check *Frequent questions* section for more information.

How to update documentation files
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The documentation uses python sphinx

.. code-block:: none

    http://www.sphinx-doc.org/en/stable/

It is possible to add new sections or modify existing ones by following the tutorials given on:

.. code-block:: none

    http://www.sphinx-doc.org/en/stable/tutorial.html

You can build again the documentation by typing:

.. code-block:: none

    cd ~/autonomous_bicycle_ws/src/autonomous_bicycle/docs
    make html

If you want to visualize the results, please open the file *index.html* on a web browser.

.. code-block:: none

    autonomous_bicycle/docs/build/html/index.html

Github and *Read the docs* offers a suitable way to update automatically the documentation on-line at each commit-push.

