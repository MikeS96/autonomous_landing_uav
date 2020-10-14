
# Autonomous landing of a UAV

ROS packages developed for the autonomous landing of a UAV on a stationary platform.

<div  align="center">
<img src="./images/land_gif.gif" width="480" />
</div>

## Description

The autonomous landing system has been tested in Simulation with Gazebo and with a modified DJI F450 with an onboard computer. The workflow of the system is a simulated environment is as follows. The system is launched in Gazebo and communicated with the Firmware of PX4. Then, the vehicle takeoff from the ground and moves to a position where the landing platform is visible. The detection module starts working and with a feature-based detector and a Kalman Filter, the landing pad is tracking thoroughly. Once the first estimation of the landing platform is made, the landing controller begins to work and moves the vehicle towards the center of the platform while it is descending. Finally, the vehicle lands and ends its mission.

<div  align="center">
<img  src="./images/world.png" width="330">
</div>

This system can be used in more complex tasks where the landing phase wants to be automated. Precision agriculture, Patrolling and building inspection are just few examples where a system like this might be used to land the vehicle.

There are three main packages that compose this project, these are:

 1. mavros_off_board
 2. object_detector
 3. drone_controller

In the package *mavros_off_board* are the launch files, world file, description files (urdf, xacro, sdf) and basic scripts to control the aircraft. The package *object_detector* is the detection and tracking (Kalman Filter) pipeline of the system, this module allows the tracking of a landing template. Finally, *drone_controller* has the proportional and PID controllers developed to land the vehicle based on the estimations made with the *object_detector* package.

<div  align="center">
<img  src="./images/uav.png" width="330">
</div>

## Installation 

To use these packages, [install OpenCV ](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), [ROS melodic](http://wiki.ros.org/melodic/Installation), [Gazebo 9](http://gazebosim.org/tutorials?tut=install_ubuntu) and the [SITL (master) ](https://dev.px4.io/master/en/simulation/) provided by PX4 in Ubuntu 16.04.

This project has dependencies in other ROS packages which are shown below. To install [mavros](http://wiki.ros.org/mavros) and [find_object_2d](http://wiki.ros.org/find_object_2d) ROS packages use the following commands.

    $ sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
    $ sudo apt-get install ros-melodic-find-object-2d
    
Once the PX4 SITL is installed, create your own model of the F450 model with the files provided in `mavros_off_board/urdf` and `mavros_off_board/sdf`. The instruction and steps are explained in this [thread](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/3). The steps are listed below.

1.  Create a folder under `Tools/sitl_gazebo/models` for the F450 model called *quad_f450_camera*
2.  Create the following files under `Tools/sitl_gazebo/models/quad_f450_camera`: model.config and quad_f450_camera.sdf (The sdf file and model.config is located  in `mavros_off_board/sdf`). Additionally, create the folder *meshes* and *urdf* and add the files in  `mavros_off_board/urdf`,  `mavros_off_board/meshes`
3.  Create a world file in `Tools/sitl_gazebo/worlds` called grass_pad.world (file located  in `mavros_off_board/worlds`)
4.  Create an airframe file under `ROMFS/px4fmu_common/init.d-posix/airframes` (This can be based off the iris or solo airframe files), give it a number (for example 1076) and name it 1076_quad_f450_camera
5.  Add the airframe name (quad_f450_camera) to the file `platforms/posix/cmake/sitl_target.cmake` in the command _set(models …_

Finally, add the three ROS packages to your catkin_ws and compile the project with `catkin_make`.

## Usage
The system has four launch files embedded in *mavros_off_board* and these are used to launch the vehicle, the functionality of each launcher is:

 - **posix_sitl.launch** It launches PX4 SITL in Gazebo
 - **mavros_posix_sitl.launch** This launch file launches Mavros, PX4 SITL and Gazebo. This launch file allows the control of the vehicle with ROS. The model launched is in sdf format.
 - **mavros_rviz.launch** This launch file is only for visualization purpose and shows the vehicle in Gazebo and Rviz. The model launched is in urdf format.
 - **urdf_launcher.launch** This launch file launches Mavros, PX4 SITL, Rviz and Gazebo. This launch file allows the control of the vehicle with ROS. The model launched is in xacro format.

The frames and TF of the vehicle in Rviz are shown in the next image.

<div  align="center">
<img src="./images/rviz_gif.gif" width="480" />
</div>

Use the launch files based on your own needs. If you need only simulate the vehicle use **mavros_posix_sitl.launch**, but if visualization in Rviz is also needed use **urdf_launcher.launch**. 

With the package **mavros_off_board**  launch the system in a simulated environment 

    cd Firmware  
    DONT_RUN=1 make px4_sitl_default gazebo  
    source ~/catkin_ws/devel/setup.bash  
    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default  
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)  
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
    roslaunch mavros_off_board mavros_posix_sitl.launch

This will deploy the Gazebo world created with the UAV, allowing further iteration of the vehicle.

Once the simulation is running,  Takeoff the vehicle and start its teleoperation with

    $ rosrun mavros_off_board offb_node
    $ rosrun mavros_off_board teleop_node_pos

Move the vehicle with the keyboard along the simulation space and locate it above the landing pad.

To use the detection pipeline in **object_detector** use 

    $ roslaunch object_detector simu.launch

This will start the detection module of the system and track the landing platform as shown by the image below.
 
 <div  align="center">
<img  src="./images/kf.png" width="330">
</div>

To land the vehicle use the **drone_controller** package. The process variable of the controller is the output of the detection pipeline. Do not use this package without the detector.  

    $ rosrun drone_controller pid_controller_final 

The variables controlled are velocity in X and Y, the yaw rate and the position in Z of the quad-rotor.

This work used the find_object_2d package developed by introlab, the citation can be seen below

> @misc{labbe11findobject,
   Author = {{Labb\'{e}, M.}},
   Howpublished = {\url{http://introlab.github.io/find-object}},
   Note = {accessed 2019-04-02},
   Title = {{Find-Object}},
   Year = 2011
}

This work was done as BEng degree project entitled "Autonomous landing system for a UAV on a ground vehicle" in "Universidad Autonóma de Occidente", Colombia. 

## Citation

If you want to use this repo, please cite as.

> @misc{MikeS96,
  author = {Saavedra-Ruiz M, Pinto A, Romero-Cano V},
  title = {Autonomous landing system for a UAV on a ground vehicle},
  year = {2019},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/MikeS96/autonomous_landing_uav}},
}
