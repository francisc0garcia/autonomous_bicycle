Tutorials related with simulation
=================================

This set of tutorials covers the process of simulation, recording and executing online pose estimation using simulated environment in Gazebo.
This tutorial assumes that the autonomous_bicycle project was successfully installed, compiled and tested according to the installation guide.

How to run the simulation of a bicycle
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This sections explains how to start the simulation and interact with the bicycle by changing its rear wheel velocity.

First, change the folder path, then, source the command window and finally run the launch file as follows:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle

A RVIZ-window with the 3D model of the bicycle and a RQT-window with plugins will be open.
Using the Dynamic reconfigure plugin in RQT-window, it is possible to change the rear wheel velocity
by clicking the "bicycle_interaction" option, and then, using the scrollbar, select the desired speed.

To reset the position and velocity to zero, please set the **velocity = -1**

How to record data from simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To record new simulated data-sets in ROSBAG files, it is necessary to run first the simulation environment following the previous tutorial.

As soon as the simulation is running, it is possible to record data following the next procedure:

Open a new command-line console, source it and run the recorder by typing:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle bicycle_record_sequence.launch bag_file:=file_name_new_simulation

To define the file name, use the argument **bag_file**.

For stop recording, please press simultaneously **[Ctrl + C]** on the command-line console.

By default, the recorder saves new data-sets on folder: **/bags/simulations/**

However, you can select a different folder by passing the argument **local_path** as it can be seen on next example:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle bicycle_record_sequence.launch bag_file:=file_name_new_simulation local_path:=bags/tests/

It is also possible to define an absolute path for storing the recorded file, by setting the argument **bag_path** directly.

How to play recorded simulated data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the simulated experiments have been recorded, it is possible to reproduce them as follows:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle bicycle_play_sequence.launch bag_file:=file_name_new_simulation

To define the file name, please change the argument **bag_file**.

A RVIZ-window as well as a RQT-window will be open, it is possible to visualize the main variables using **multiplot** plugin.
You just need to load the configuration file **config/rqt_multiplot_simulation.xml** using the graphical interface.

How to Convert recorded data into CSV/MAT for Python/MatLab interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to process data using python or Matlab, it is necessary to convert the ROSBAG file into CSV or MAT format,
A launch file take cares of such conversion by running:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle bicycle_rosbag2csv.launch input_format:=gazebo local_path:=bags/simulations/ bag_file:=file_name_new_simulation

- **input_format** Defines the source of the data, can be either *gazebo* or *real_data*
- **local_path** Specify the local path to the ROSBAG file
- **bag_file** Defines the file name of the ROSBAG file

As soon as the process is finished, a new folder with the name of the ROSBAG file is created, containing all CVS files (separated by topic and merged),
as well as a MAT file.

The suffix **_preprocessed** means that, filtering and sampling was applied to the original file, both (RAW and preprocessed)
CSV data are available.

How to run the online pose estimator using simulated data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Finally,  if you want to run the pose estimation filters using the recorded simulation, just execute the following commands:

.. code-block:: none

    cd ~/autonomous_bicycle_ws
    source devel/setup.bash
    roslaunch autonomous_bicycle pose_estimation.launch local_path:=bags/simulations/ bag_file:=file_name_new_simulation

It is possible to visualize in detail all state variables using the plugin **multiplot** on the RQT-window, just search and select the configuration file
**config/rqt_multiplot_pose_estimation.xml**

It is also possible to execute the pose estimation process off-line, by using the including Jupyter's notebook in python.
A separate tutorial will cover this topic.
