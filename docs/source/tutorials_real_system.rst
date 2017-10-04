Tutorials related with real bicycle experiments
===============================================

This set of tutorials covers the process of starting, recording, playing back and executing online pose estimation
using real data gathered from the measurement system installed on the bike.
This tutorial assumes that the autonomous_bicycle project was successfully installed, compiled and tested according to the installation guide.

How to start and test the system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, connect the power bank to the Raspberry PI 3 and USB external-powered Hub and
wait for 1 minute while the WiFi router is ready to setup new connections.

Take a cellphone, tablet or standard PC, and connect to the network created by the WiFi router,
by default, the name of the network and its password is **AutonomousBicycle**

For starting the measurement system, please log into the Raspberry PI 3 using SSH connection:

.. code-block:: none

    ssh ubuntu@192.168.1.201

User and password are the same: **ubuntu**

Then, execute the initialization launch file:

.. code-block:: none

    cd ~/Documents/code/autonomous_bicycle
    source devel/setup.bash
    export ROS_IP=192.168.1.201
    roslaunch autonomous_bicycle sensor_init_raspberry.launch

Then, verify the connection opening a web browser and typing the URL: **192.168.1.201:3000**
If everything is running properly, a graphical interface should appears and
the sensor's state must be available.

If you want to connect a PC running Linux with the bike for visualization and debugging,
please type in a new command-line window:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    export ROS_IP=192.168.1.200
    export ROS_MASTER_URI=http://192.168.1.201:11311
    roslaunch autonomous_bicycle sensor_init_real.launch

Notice that you must change your **ROS_IP** accordingly, if you do not know it, type **ifconfig**.

If the connection has been properly established, a RVIZ-windows should open with the bicycle 3D model,
showing on-line the steering and lean angle as well as the camera image.

If GPS data is not available (Marked as Red color), please go to the tutorials for configuration and set them up.

How to record experiments with the bicycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, start the bicycle measurement system as indicated before, and then follows the sequence:

- Open a web browser and search for the following URL: **192.168.1.201:3000**
- Go to the panel UI **Calibration** and set-up the IMU offset to the current bicycle pose.
- Go to the panel UI **Recording**, set an experiment's name, and start recording.
- Ride the bicycle and, when the experiment is finished, press the stop button.
- Final ROSBAG file is placed into the Raspberry PI 3, inside the **autonomous bicycle package** folder.

If GPS data is not available (Marked as Red color), please go to the tutorials for configuration and set them up.

How to play previously recorded experiments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The first step is to copy the ROSBAG from Raspberry into a PC running linux.
You can transfer them using a standard SFTP application, such Filezilla,
and searching into the folder **Documents/code/autonomous_bicycle** using the same SSH credentials.

Once you have copied them, **open a new command-line window** and type:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle sensor_init_real.launch load_rosbag:=True rviz_config:=rviz_measurement_data bag_file:=07_09_2017/ExperimentBicycle070917R03

- **load_rosbag** Load an existing ROSBAG file
- **rviz_config** Selects the corresponding RVIZ configuration file (visualization setup)
- **bag_file** Select the bag file
- **bag_path** Define the absolute path where the ROSBAG file was placed

If everything goes well, a RVIZ-window with the map, 3D model of the bicycle and trajectories should appear into screen.

How to Convert recorded data into CSV/MAT for Python/MatLab interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to process data using python or Matlab, it is necessary to convert the ROSBAG file into CSV or MAT format,
A launch file take cares of such conversion by running:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle bicycle_rosbag2csv.launch input_format:=real_data local_path:=bags/experiments/ bag_file:=file_name_new_simulation

- **input_format** Defines the source of the data, can be either *gazebo* or *real_data*
- **local_path** Specify the local path to the ROSBAG file
- **bag_file** Defines the file name of the ROSBAG file

As soon as the process is finished, a new folder with the name of the ROSBAG file is created, containing all CVS files (separated by topic and merged),
as well as a MAT file.

The suffix **_preprocessed** means that, filtering and sampling was applied to the original file, both (RAW and preprocessed)
CSV data are available.


