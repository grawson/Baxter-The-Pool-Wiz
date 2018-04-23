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

Then run in a sourced terminal:

``` bash
cd ~/Baxter-The-Pool-Wiz
./baxter.sh sim
roslaunch baxter_pool_sim pool.launch
```

To launch the pool shot planner, run:


``` bash
roslaunch pool_planner pool_planner.launch
```


### PCL

We downloaded PCL plus a few things in order to get everything to work. 

Install oracle-java8-jdk:

``` bash
sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer
```

Install universal pre-requisites:

``` bash
sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev     
```

For PCL v1.8, Ubuntu 16.04.2 input the following:

``` bash
sudo apt -y install libflann1.8 libboost1.58-all-dev

cd ~/Downloads
wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
sudo apt-mark hold libeigen3-dev

wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
tar -xf VTK-7.1.0.tar.gz
cd VTK-7.1.0 && mkdir build && cd build
cmake ..
make                                                                   
sudo make install

cd ~/Downloads
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0 && mkdir build && cd build
cmake ..
make
sudo make install

cd ~/Downloads
rm libeigen3-dev_3.2.5-4_all.deb VTK-7.1.0.tar.gz pcl-1.8.0.tar.gz
sudo rm -r VTK-7.1.0 pcl-pcl-1.8.0
```

We needed this line too:

``` bash
sudo apt install libeigen3-dev
```
