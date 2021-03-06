## Demo

A working demo of Baxter sinking a shot can be found at:
[Demo](https://www.youtube.com/watch?v=Sl_cYK4EDLY&feature=youtu.be)

## Cloning Repo

Make sure to clone this repo into your home directory in order to user the install file. That is:

``` bash
cd ~
git clone <OUR-REPO>
```

## Install

We've provided an install file that will set up the Baxter simulator within a ROS Kinetic environment. Make sure you've got pip3, catkin, and a GitHub account.

``` bash
cd ~/Baxter-The-Pool-Wiz
./install.sh
```

Now edit `Baxter-The-Pool-Wiz/baxter.sh` and make sure the 30th ish line reads:

``` bash
ros_version="kinetic"
```

## Test Installation

Whenever running any commands, make sure you are in a terminal window and have run the commands:

``` bash
source ~/Baxter-The-Pool-Wiz/devel/setup.bash
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
```

Setup a terminal window and run:

``` bash
# Launch Baxter in an empty world
roslaunch baxter_gazebo baxter_world.launch
```

Wait for Gazebo to spin up. You should see the robot in the world. In another setup terminal window, run:

``` bash
# Enable the robot
rosrun baxter_tools enable_robot.py -e      
```

Then:

``` bash
# run test
rosrun baxter_examples joint_velocity_wobbler.py  
```

The Baxter should be waving its arms :)  
Now to test MoveIt!, stop the wobbler script and run:

``` bash
# Start trajectory controller
rosrun baxter_interface joint_trajectory_action_server.py
```

In another setup window:

``` bash
# Start Rviz MoveIt! plugin
roslaunch baxter_moveit_config baxter_grippers.launch

```

Wait until you see the message `You can start planning now!`. Move the gripper around using the tool overlaying the gripper. Once a destination pose is selected (marked in orange), go to the `planning` tab and click `plan`. You should see the animated trajectory.

## Update Repos

Keep dependencies in `src/dep` up to date with command:

```bash
gitman update
```

## Pool Simulation

Make sure to put the contents of the `meshes` directory into your gazebo models directory located at `~/.gazebo/models`

Whenever running any commands, make sure you are in a terminal window and have run the commands:

``` bash
source ~/Baxter-The-Pool-Wiz/devel/setup.bash
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
```

``` bash
# launch sim
roslaunch baxter_pool_sim pool.launch   

# Enable the robot
rosrun baxter_tools enable_robot.py -e 

# Start trajectory controller
rosrun baxter_interface joint_trajectory_action_server.py   

# Start Rviz MoveIt! plugin
roslaunch baxter_moveit_config baxter_grippers.launch

# moveit pose planner
roslaunch pool_planner pose_planner.launch
```
