

# Installation and environment configuration

This short tutorial is intended to be used as the installation guide for the development tool-chain of the SITL provided by PX4. The information provided here is based on the [official documentation](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html) and was created with the version `1.11`.

The operative system used is [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 18.04 (Bionic Beaver) with [ROS "Melodic"](https://dev.px4.io/v1.11/en/setup/dev_env_linux_ubuntu.html#rosgazebo) and [Gazebo 9](https://dev.px4.io/v1.11/en/simulation/gazebo.html). We will go through the files needed to install each one of these as I did.

## Setting up a basic environment

First of all, download the Firmware in `/home/<your_user>/`  with the next command.
```
git clone https://github.com/PX4/Firmware.git --recursive
```
Once it is downloaded it will create a folder called 'Firwmare'. To install the basic development dependencies do as follows.
```
cd Firwamre
bash ./Tools/setup/ubuntu.sh
```
The last lines install set up a development environment that includes [Gazebo 9](https://dev.px4.io/v1.11/en/simulation/gazebo.html) and [jMAVSim](https://dev.px4.io/v1.11/en/simulation/jmavsim.html) simulators, and the [NuttX/Pixhawk](https://dev.px4.io/v1.11/en/setup/building_px4.html#nuttx) tool-chain which are needed to use the SITL. Additionally, I encountered some errors in the past which were solved with the next commands.

    sudo apt install libgstreamer1.0-dev 
    sudo apt install gstreamer1.0-plugins-good  
    sudo apt install gstreamer1.0-plugins-bad  
    sudo apt install gstreamer1.0-plugins-ugly

You can verify that the installation was successful by running.
```
$arm-none-eabi-gcc --version
```
The result if everything was right should be:
```
 arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
 Copyright (C) 2017 Free Software Foundation, Inc.
 This is free software; see the source for copying conditions.  There is NO
 warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```
## Setting up ROS Melodic and Mavros

The next steps are needed to set up ROS Melodic and Mavros. If you do not have it yet use [ubuntu_sim_ros_melodic.sh](https://github.com/PX4/Devguide/blob/master/build_scripts/ubuntu_sim_ros_melodic.sh).  This file aims to install ROS Melodic and mavros from source. However, we will do it differently here.

First of all download the file using.


    cd ~
    wget https://github.com/PX4/Devguide/blob/master/build_scripts/ubuntu_sim_ros_melodic.sh

If you **do not have** ROS Melodic installed, open the file [ubuntu_sim_ros_melodic.sh](https://github.com/PX4/Devguide/blob/master/build_scripts/ubuntu_sim_ros_melodic.sh) and remove the lines from 64 until the end of the file. The line 64 should start as follows:

    ## Install MAVLink
    ###we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
    rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
    
    ## Build MAVROS
    ### Get source (upstream - released)
    rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
    ...
    ...

However, if you do already have ROS Melodic installed, remove from line 37 until the end. Line 37 start as follows:

    ## Get ROS/Gazebo
    sudo apt install ros-melodic-desktop-full -y
    ## Initialize rosdep
    sudo rosdep init
    rosdep update
    ## Setup environment variables
    rossource="source /opt/ros/melodic/setup.bash"
    if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
    else echo "$rossource" >> ~/.bashrc; fi
    eval $rossource
    ...
    ...

Once you have confirmed that ROS is properly installed and everything is working as expected, it is time to install the final ROS dependencies. To install **Mavros**, run the following lines of code:

```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

This project has dependencies in other ROS packages which are shown below. To install [find_object_2d](http://wiki.ros.org/find_object_2d) and [vision_opencv](http://wiki.ros.org/vision_opencv) ROS packages using the following commands.

```
sudo apt-get install ros-melodic-find-object-2d
sudo apt-get install ros-melodic-vision-opencv
```

### Models configuration

Once the PX4 SITL is installed, create your own model of the F450 model with the files provided in `mavros_off_board/urdf` and `mavros_off_board/sdf`. The instruction and steps are explained in this [thread](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/3). The steps are listed below.

1.  Create a folder under `Tools/sitl_gazebo/models` for the F450 model called *quad_f450_camera*
2.  Create the following files under `Tools/sitl_gazebo/models/quad_f450_camera`: model.config and quad_f450_camera.sdf (The sdf file and model.config is located  in `mavros_off_board/sdf`). Additionally, create the folder *meshes* and *urdf* and add the files in  `mavros_off_board/urdf`,  `mavros_off_board/meshes`
3.  Create a world file in `Tools/sitl_gazebo/worlds` called grass_pad.world (file located  in `mavros_off_board/worlds`)
4.  Create an airframe file under `ROMFS/px4fmu_common/init.d-posix/airframes` (This can be based off the iris or solo airframe files), give it a number (for example 1076) and name it 1076_quad_f450_camera
5.  Add the airframe name (quad_f450_camera) to the file `platforms/posix/cmake/sitl_target.cmake` in the command that starts with `set(models …`

Finally, add the three ROS packages to your catkin_ws and compile the project with `catkin_make`.

At this point everything should be up and running to start playing with the SITL of PX4. 
