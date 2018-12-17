# swarm_stage_ros

A ROS package that automatically configures a swarm robot simulator, aiming the simulation of large-scale swarms and minimizing the computational cost, with homogeneous and heterogeneous robots, different perceptions systems, to the main tasks of robot swarm.

# Installation

The swarm_stage_ros package can be installed on your workspace:

> cd /home/user/catkin_ws/src
> git clone https://github.com/VivianCremerKalempa/swarm_stage_ros
> cd ..
> catkin_make

To run your package, you just need this command:

> roslaunch swarm_stage_ros swarm_stage.launch

Note that roslaunch starts ROS automatically.

At the end of processing, two files are generated: custom.world and controller.launch. The custom.world file has all the necessary specification for the scenario to be run on the Stage, for example:

> include "/home/vivian/catkin_ws/src/swarm_stage_ros/launch/include/map.inc"
> include "/home/vivian/catkin_ws/src/swarm_stage_ros/launch/include/robots.inc"
> include "/home/vivian/catkin_ws/src/swarm_stage_ros/launch/include/sensors.inc"
> floorplan (name "cave" size [16.000 16.000 0.800] pose [0 0 0 0] bitmap "/home/vivian/catkin_ws/src/swarm_stage_ros/launch/map/image.png")
> turtlebot (name "r0" pose [0.00 0.00 0 0 ]  color "red")
> roomba (name "r1" pose [-0.30 0.30 0 0 ]  color "blue")
> roomba (name "r2" pose [-0.30 -0.30 0 0 ]  color "blue")
> roomba (name "r3" pose [-0.60 0.60 0 0 ]  color "blue")
> roomba (name "r4" pose [-0.60 -0.60 0 0 ]  color "blue")
> roomba (name "r5" pose [-0.90 0.90 0 0 ]  color "blue")
> roomba (name "r6" pose [-0.90 -0.90 0 0 ]  color "blue")
> roomba (name "r7" pose [-1.20 1.20 0 0 ]  color "blue")
> roomba (name "r8" pose [-1.20 -1.20 0 0 ]  color "blue")
> roomba (name "r9" pose [-1.50 1.50 0 0 ]  color "blue")
> roomba (name "r10" pose [-1.50 -1.50 0 0 ]  color "blue")

The controller.launch file is generated in order to run the environment that has just been created:

> <?xml version="1.0"?>
> <launch>
>   <!-- ROSCORE -->
> 	<master auto="start"/>
>   <!--  Stage Simulator -->
>   	<node pkg="stage_ros" type="stageros" name="stageros" args="$(find swarm_stage_ros)/launch/custom.world" respawn="true" output="log"/> 
> </launch>


The swarm_stage.launch requires as a parameter the arguments passed by the swarm.yaml file.

