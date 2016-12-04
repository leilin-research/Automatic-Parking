# Automatic Parking in simulation

This project utilized reinforcement learning algorithm (Q-learning with epsilon greedy algorithm)

## Installation
Requirements:

* Python 2.7
* matplotlib
* numpy
* [ROS indigo (or kinetic)](http://www.ros.org/install/)

After you installed these libraries and ROS, follow the following steps:

```bash
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-controller-manager

mkdir ~/parking
cd ~/parking
git clone https://github.com/CTTC/Automatic-Parking.git

mkdir -p ~/catkin_ws/src
mv ackermann_model/* ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Done! Now you can play with the ackermann car in Gazebo by:
```bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
roslaunch ackermann_drive_teleop ackermann_drive_keyop.launch
```


## Run Demos

### Demo in Matplotlib
```
cd ~/parking/q_learning/matplotlib_sim/demo
python agent.py
```

### Demo in Gazebo
Start running the agent in Gazebo first:
```bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
```
Then run the demo
```
cd ~/parking/q_learning/gazebo_sim/demo
python agent.py
```
