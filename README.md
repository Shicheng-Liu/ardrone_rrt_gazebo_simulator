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
Now, you can see your gazebo simulated world. Then, run the controller:
```
roslaunch ardrone_rrt ardrone_rrt.launch
```
You will see a Qt window which is the same with the one in _ardrone_tutorials_ since this work is based on _ardrone_tutorials_. You can still use _keyboard_controller_ to command the drone to move, Here is the control keys:<br>
W – Pitch Forward     <br>S – Pitch Backward            
A – Roll Left                                        <br>D – Roll Right
<br>
Q – Yaw Left                                        <br> E – Yaw Right
<br>
Z – Increase Altitude                                <br>C – Decrease Altitude
<br>
Y – Takeoff                                         <br> H – Land
<br>
SPACEBAR – EMERGENCY STOP
<br>
T – Trajectory
<br>
R – RRT
<br><br>
You can see two functions, _Trajectory_ and _RRT_, are added.<br>
**Attention**:  _Trajectory_ can only be used in _nrsl.world_ environment, and _RRT_ can be used in both environments but better in _nrsl_with_obstacles.world_ environment.<br>
<br> Here is what it goes:

![image](https://github.com/Shicheng-Liu/ardrone_rrt_gazebo_simulator/blob/master/Trajectory.gif)

The drone follows the assigned trajectory(_rectangle_).

![image](https://github.com/Shicheng-Liu/ardrone_rrt_gazebo_simulator/blob/master/RRT.gif)

The drone will fly to the assigned original place first and then you can click to assign the target position.
<br> This alogorithm actually uses RRT* and has 2000 iterations(to save time, you can increase the iterations to make the path looker). The drone will follow the path once the path is drawn on pygame.
