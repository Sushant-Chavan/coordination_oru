#!/bin/bash

cd ../
# Set the current directory as the root dorectory for all installations
ROOT_DIR=$(pwd)

# Install figlet for ASCII fonts in console during setup
sudo apt install -y figlet

#--------------------------------------------------
# Install ROS
#--------------------------------------------------
#figlet -t ============
#figlet -t Install ROS
#figlet -t ============
#cd coordination_oru/
#./install_ros.sh

#--------------------------------------------------
# Setup OMPL
#--------------------------------------------------
figlet -t ============
figlet -t Install OMPL
figlet -t ============

cd $ROOT_DIR
git clone https://github.com/Sushant-Chavan/ompl.git
cd ompl/
git checkout CustomBuildSettings
./install-ompl-ubuntu.sh.in


#--------------------------------------------------
# Install SMPL
#--------------------------------------------------
figlet -t ============
figlet -t Install SMPL
figlet -t ===========
cd $ROOT_DIR
mkdir -p SMPL/catkin_ws/src
cd SMPL
## Clone and install SBPL
git clone https://github.com/sbpl/sbpl
cd sbpl 
mkdir build 
cd build 
cmake .. 
make 
sudo make install
## Create a catkin workspace
cd ../../catkin_ws/
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
catkin init
catkin build
cd src 
## Clone and install leatherman
#sudo apt install -y ros-kinetic-moveit-msgs ros-kinetic-octomap ros-kinetic-octomap-ros
#git clone https://github.com/aurone/leatherman
#source ~/.bashrc
#catkin build
#source ../devel/setup.bash
## Clone and install SMPL
git clone https://github.com/Sushant-Chavan/smpl.git
#rosdep install --from-paths smpl -i -y
catkin build smpl smpl_ompl_interface
source ../devel/setup.bash
## Build and install custom smpl and smpl_ompl_interface packages
mkdir -p smpl/smpl/build
mkdir -p smpl/smpl_ompl_interface/build
cd smpl/smpl/build 
rm -rf * 
cmake .. 
make 
sudo make install 
sudo ldconfig
cd ../../smpl_ompl_interface/build 
rm -rf * 
cmake .. 
make 
sudo make install 
sudo ldconfig 


#--------------------------------------------------
# Clone coordination_oru
#--------------------------------------------------
figlet -t ===========
figlet -t Install Coordination Oru
figlet -t ===========
cd $ROOT_DIR

# Install MRPT
sudo apt install -y mrpt-apps libmrpt-dev 

#git clone https://github.com/Sushant-Chavan/coordination_oru.git
cd coordination_oru/

## Build the GraphML generation tool
mkdir -p graphml_generator/build
cd graphml_generator/build/
rm -rf *
cmake ..
make

# Build and install DWT tool
mkdir -p $ROOT_DIR/coordination_oru/generators/logging/DynamicTimeWarping/build
cd $ROOT_DIR/coordination_oru/generators/logging/DynamicTimeWarping/build
rm -rf *
cmake ..
make 
sudo make install 
sudo ldconfig

# Build and install custom OMPL planner 
mkdir -p $ROOT_DIR/coordination_oru/OmplPlanner/build
cd $ROOT_DIR/coordination_oru/OmplPlanner/build
rm -rf *
cmake ..
make 
sudo make install 
sudo ldconfig

# Install python requirements
cd $ROOT_DIR/coordination_oru
sudo apt install -y python3-pip python3-tk
sudo -H pip3 install --upgrade pip setuptools
sudo pip3 install -r requirements.txt


cd $ROOT_DIR


figlet -t Setup Complete





