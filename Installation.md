
# Installation

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
At this point everything should be up and running to start playing with the SITL of PX4. 

## Testing

Run the following command to test the [jMAVSim Simulator](https://dev.px4.io/v1.11/en/setup/building_px4.html).
```
cd ~
cd Firwamre
make px4_sitl jmavsim
```
The result is the UAV in the jMAVSim simulator.

<div  align="center">
<img src="https://dev.px4.io/v1.11/assets/console_jmavsim.png" width="480" />
</div>
The drone can be flown by typing:

    pxh> commander takeoff

<div  align="center">
<img src="https://dev.px4.io/v1.11/assets/jmavsim_first_takeoff.png" width="480" />
</div>

To test the SITL within Gazebo use:

```
make px4_sitl gazebo
```

<div  align="center">
<img src="https://dev.px4.io/v1.11/assets/simulation/gazebo/gazebo_follow.jpg" width="480" />
</div>

The drone can be flown by typing:

    pxh> commander takeoff

From now on you can continue following the steps provided [here](https://github.com/MikeS96/autonomous_landing_uav).

**Note** To automatically install the ROS dependencies needed for the custom package, please use the following lines (Remember to set the [packages](https://github.com/MikeS96/autonomous_landing_uav) in your catkin workspace)

    cd catkin_ws
    source devel/setup.bash
    rosdep install mavros_off_board
    
If everything goes right the result should be as follows:
   
    #All required rosdeps installed successfully


