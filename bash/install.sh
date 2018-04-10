# setup workspace
sudo rm -r ~/Baxter-The-Pool-Wiz
mkdir -p ~/Baxter-The-Pool-Whiz/src
source /opt/ros/kinetic/setup.bash
cd ~/Baxter-The-Pool-Whiz
catkin build

# install Baxter SDK dependencies
sudo apt-get update
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers

# Install Baxter SDK
cd ~/Baxter-The-Pool-Whiz/src
wstool init .
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall
wstool update

# Build workspace
source /opt/ros/kinetic/setup.bash
cd ~/Baxter-The-Pool-Whiz
catkin build

# Configure Baxter Communication
cd ~/Baxter-The-Pool-Whiz
wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh

# Edit file to set ros_version="kinetic"
vim baxter.sh
. baxter.sh

# Install prerequisites
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser

# Install Baxter simulator
cd ~/Baxter-The-Pool-Whiz/src
wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall
wstool update

# Build workspace
source /opt/ros/kinetic/setup.bash
cd ~/Baxter-The-Pool-Whiz
catkin build
cp src/baxter/baxter.sh .

# Edit ros ros_version again
vim baxter.sh
