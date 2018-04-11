## Cloning Repo

Make sure to clone this repo into your home directory in order to user the install file. That is:

``` bash
cd ~
git clone <OUR-REPO>
```

## Install

We've provided an install file that will set up the Baxter simulator within a ROS Kinetic environment.

``` bash
cd ~/Baxter-The-Pool-Wiz
./install.sh
```

## Test Installation

Make sure to always source the workspace when opening a new terminal window.

``` bash
source ~/Baxter-The-Pool-Wiz/devel/setup.bash
```

Source a terminal window and run:

``` bash
# Launch Baxter in an empty world
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch
```

Wait for Gazebo to spin up. You should see the robot in the world. In another sourced terminal window, run:

``` bash
# Enable the robot
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
rosrun baxter_tools enable_robot.py -e      

# run test
rosrun baxter_examples joint_velocity_wobbler.py  
```

The Baxter should be waving its arms :)  
Now to test MoveIt!, stop the wobbler script and run in the same window:

``` bash
# Start trajectory controller
rosrun baxter_interface joint_trajectory_action_server.py
```

In another sourced terminal:

``` bash
# Start Rviz MoveIt! plugin
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
roslaunch baxter_moveit_config baxter_grippers.launch

```

Wait until you see the message `You can start planning now!`. Move the gripper around using the tool overlaying the gripper. Once a destination pose is selected (marked in orange), go to the `planning` tab and click `plan`. You should see the animated trajectory.

## Pool Simulation

Make sure to put the contents of the `meshes` directory into your gazebo models directory located at `~/.gazebo/models`

Then run in a sourced terminal:

``` bash
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
roslaunch baxter_pool_sim pool.launch
```
