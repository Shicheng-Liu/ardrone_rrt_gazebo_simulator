ardrone_rrt_gazebo_simulator
=
This work is based on [tum_simulator](https://github.com/tum-vision/tum_simulator) and [ardrone_tutorials](https://github.com/mikehamer/ardrone_tutorials). In the _drone_controller_gazebo.py_, a client is added to receive pose data of the simulated ardrone from gazebo so that we can command the drone to move to assigned postion. If you are interested in rrt algorithm, you can watch the python code [here](https://github.com/Shicheng-Liu/ardrone_rrt)
<br>

Systems Work on
-
Ubuntu 16 &nbsp;    &nbsp;  ROS kinetic<br>
Ubuntu 14 &nbsp;    &nbsp;  ROS indigo
<br>

ROS Package Installation
-
First, we need to create a workspace:
```
mkdir ardrone_simulator
```
Second, we need to clone this repository in your ROS workspace:
```
cd ardrone_simulator
git clone https://github.com/Shicheng-Liu/ardrone_rrt_gazebo_simulator.git
```
Then, you will find there is a folder in your workspace called _ardrone_rrt_gazebo_simulator_. **Change the folder's name to src!!!!**
After that, you can:
```
catkin_make
```
And don't forget to source your workspace!

How to use this 
-
Here are two simulation environment: _nrsl.world_ and _nrsl_with_obstacles.world_.
<br> To open the simulated world, you need to run:
```
roslaunch cvg_sim_gazebo nrsl.launch
```
or 
```
roslaunch cvg_sim_gazebo nrsl_with_obstacles.launch
```
