#!/bin/bash
sudo apt update && sudo apt upgrade -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
sudo apt-get install -y python3-catkin-tools
sudo apt install -y python3-wstool
sudo apt install -y ros-noetic-ros-control
sudo apt install -y python3-pip
pip3 install catkin_pkg
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
cd src
git clone https://github.com/FIRAAir/FIRA-Air-Simulator.git
cd ..
catkin_make
source ~/.bashrc
rosrun fira_challenge_env model_update.py
