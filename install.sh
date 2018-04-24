# install gitman
pip3 install gitman

# setup workspace
source /opt/ros/kinetic/setup.bash
cd ~/Baxter-The-Pool-Wiz
catkin build

# install Baxter SDK dependencies
sudo apt-get update
sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers

# Install prerequisites
sudo apt-get install gazebo7 ros-kinetic-qt-build ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-ros-control ros-kinetic-control-toolbox ros-kinetic-realtime-tools ros-kinetic-ros-controllers ros-kinetic-xacro python-wstool ros-kinetic-tf-conversions ros-kinetic-kdl-parser

# Install MoveIt
sudo apt install ros-kinetic-moveit*

# install git repos
gitman install --force

# Configure Baxter Communication
cd ~/Baxter-The-Pool-Wiz
wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
chmod u+x baxter.sh

# Build workspace
source /opt/ros/kinetic/setup.bash
cd ~/Baxter-The-Pool-Wiz
catkin build
rm baxter.sh*
cp src/dep/baxter/baxter.sh .
