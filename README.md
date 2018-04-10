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
rosrun baxter_tools enable_robot.py -e      
```

And in another:

``` bash
# run test
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
rosrun baxter_examples joint_velocity_wobbler.py  
```

The Baxter should be waving its arms :)


## Pool Simulation

Make sure to put the contents of the `meshes` directory into your gazebo models directory located at `~/.gazebo/models`

Then run:

``` bash
source ~/Baxter-The-Pool-Wiz/deve/setup.bash
./baxter.sh sim
roslaunch baxter_pool_sim pool.launch
