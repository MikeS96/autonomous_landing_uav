
# Testing

Run the following command to test the [jMAVSim Simulator](https://docs.px4.io/master/en/dev_setup/building_px4.html).
```bash
cd ~
cd PX4-Autopilot
make px4_sitl jmavsim

```
The result is the UAV in the jMAVSim simulator.

<div  align="center">
<img src="https://dev.px4.io/v1.11/assets/console_jmavsim.png" width="480" />
</div>
The drone can be flown by typing:

```bash
pxh> commander takeoff
```

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

```bash
pxh> commander takeoff
```
From now on you can continue following the steps provided [here](https://github.
com/MikeS96/autonomous_landing_uav/Usage.md).

**Note** To automatically install the ROS dependencies needed for the custom package, please use the following lines (Remember to set the [packages](https://github.com/MikeS96/autonomous_landing_uav) in your catkin workspace)

```bash
cd catkin_ws
source devel/setup.bash
rosdep install mavros_off_board
``` 

If everything goes right the result should be as follows:
   
```bash
#All required rosdeps installed successfully
```