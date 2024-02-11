<h1 align="center"> Fundamentals of Robotics Project</h1>
<h4 align="center"> Matteo Grisenti - Angelo Nutu - Mattia Barborini - Filippo Adami </h3>

## Requirements

In order to be able to run the project, it's required to follow the [locosim procedure](https://github.com/mfocchi/locosim) up until the locosim repository and catkin setup

## How to run

After setting up locosim move into the workspace folder and clone the repo:
~~~ bash
cd ~/ros_ws/src
git clone git@github.com:barbomattia/Robotica.git
~~~

Following this step, we should generate a new simulation world, following these steps very closely (the paths are important!):

0. open the docker or VM 
1. move to the appropriate path
~~~  bash
cd ~/ros_ws/src 
~~~
2. run 
~~~  bash
python3 Robotica/vision_planner/src/generate_world.py
~~~
3. open `locosim/ros_impendance_controller_ur5.launch` and modify line 12 to `<arg name="world_name default="bricks.world"/>`
4. in `locosim/robot_controller/base_controllers/ur5_generic.py` comment lines 71-72-73 relating the world name and add
~~~ python
self.world_name = 'bricks.world'
~~~
5. modify `~/.bashrc` by adding at the eof `export GAZEBO_MODEL_PATH=/home/{USER}/ros_ws/src/Robotica/vision_planner/worlds/models:$GAZEBO_MODEL_PATH`
6. run
~~~ bash
source ~/.bashrc
rm ~/ros_ws/install
cd ~/ros_ws
catkin_make install
~~~

And you should be able to find a new world with randomly placed blocks

Finally, open 3 other terminals and run:
~~~ bash
rosrun motion_planner inverse_kinematic_node 

rosrun vision_planner main.py

rosrun task_planner state_machine_task_node 

python3 -i ~/ros_ws/src/locosim/robot_controller/base_controllers/ur5_generic.py
~~~

<h5 align="center"> <a href="https://github.com/barbomattia/Robotica/blob/main/Report.pdf"> Read the full report </a> - <a href="https://drive.google.com/file/d/1hGNZJnl_iq87XpLlgdKWfP9evemomHeB/view?usp=drivesdk">Watch the demo</a></h5>
