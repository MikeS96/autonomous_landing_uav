

# Installation and environment configuration

This short tutorial is intended to be used as the installation guide for the development tool-chain of the SITL provided by PX4. The information provided here is based on the [official documentation](https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html) and was created with the git hash `cab477d71550558756509ad3a6ffcbebbbbf82b1`.

The operative system used is [Ubuntu Linux LTS](https://wiki.ubuntu.com/LTS) 20.04 (Focal Fossa) with [ROS "Noetic"](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Gazebo 11](https://dev.px4.io/v1.11/en/simulation/gazebo.html). We will go through the files needed to install each one of these as I did.

## Setting up a basic environment

Install ROS Noetic (which comes by default with Gazebo 11) by following [ROS Noetic installation](http://wiki.ros.org/noetic/Installation/Ubuntu).

Then download the PX4-Autopilot in `/home/$USER/`  with the next command.
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git checkout cab477d71550558756509ad3a6ffcbebbbbf82b1
```
Once it is downloaded it will create a folder called `PX4-Autopilot`. To install the basic development dependencies do as follows.
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
The last lines install a development environment that includes [Gazebo 11](https://dev.px4.io/v1.11/en/simulation/gazebo.html) and [jMAVSim](https://docs.px4.io/master/en/simulation/jmavsim.html) simulators, and the [NuttX/Pixhawk](https://docs.px4.io/master/en/dev_setup/building_px4.html) tool-chain which are needed to use the SITL. Additionally, I encountered some errors in the past which were solved with the next commands.

```bash
sudo apt install libgstreamer1.0-dev 
sudo apt install gstreamer1.0-plugins-good  
sudo apt install gstreamer1.0-plugins-bad  
sudo apt install gstreamer1.0-plugins-ugly
```

You can verify that the installation was successful by running.
```
arm-none-eabi-gcc --version
```
The result if everything was right should be:
```
arm-none-eabi-gcc (GNU Arm Embedded Toolchain 9-2020-q2-update) 9.3.1 20200408 (release)
Copyright (C) 2019 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```
## Setting up Mavros and GeographicLib

The next steps are needed to set up Mavros and GeographicLib. This steps are based on this [guide](https://docs.px4.io/v1.12/en/ros/mavros_installation.html) from the official PX4 documentation.

Install Mavros.

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```

This project has dependencies in other ROS packages which are shown below. To install [find_object_2d](http://wiki.ros.org/find_object_2d) and [vision_opencv](http://wiki.ros.org/vision_opencv) ROS packages using the following commands.

> Note: Please consider installing OpenCV from source (before running these commands) to enable SIFT and SURF as these 
> produced the best results 
> over the project.

```
sudo apt-get install ros-noetic-find-object-2d
sudo apt-get install ros-noetic-vision-opencv
```


### Models configuration

Once the PX4 SITL is installed, create your own model of the F450 model with the files provided in `mavros_off_board/urdf` and `mavros_off_board/sdf`. The instruction and steps are explained in this [thread](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/3). The steps are listed below.

1. Create a folder under `Tools/sitl_gazebo/models` for the F450 model called *quad_f450_camera*
2. Create the following files under `Tools/sitl_gazebo/models/quad_f450_camera`: model.config and quad_f450_camera.sdf (The sdf file and model.config is located  in `mavros_off_board/sdf`). Additionally, create the folder *meshes* and *urdf* and add the files in  `mavros_off_board/urdf`,  `mavros_off_board/meshes`
3. Create a world file in `Tools/sitl_gazebo/worlds` called grass_pad.world (file located  in `mavros_off_board/worlds`)
4. Create an airframe file under `ROMFS/px4fmu_common/init.d-posix/airframes` (This can be based off the iris or 
    solo airframe files), give it a number (for example 1076) and name it 1076_quad_f450_camera. (You can find the 
    airframe file at `mavros_off_board/files`)
5. Add the airframe file, for example `1076_quad_f450_camera` to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` at the bottom of the list starting with `px4_add_romfs_files(...`
6. Add the airframe name (quad_f450_camera) to the file `platforms/posix/cmake/sitl_target.cmake` in the command that starts with `set(models …` as-well as the world file `grass_pad` to the line starting with `set(worlds...`
7. Copy the models located at `mavros_off_board/worlds/gazebo` into `.gazebo/models`. In case the folder 
   `models` does not exist within `.gazebo`, create it first.

Finally, add the three ROS packages to your catkin_ws and compile the project with `catkin_make`.

At this point everything should be up and running to start playing with the SITL of PX4. 


