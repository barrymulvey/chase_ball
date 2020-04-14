# chase_ball
Robot which detects and chases a white-coloured ball using camera and lidar sensors. 

*description*


## Project Folder

*folders*


## Building

```
# Source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# Create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws

# Clone the driver
$ git clone https://github.com/mulbarry/chase_ball.git src/chase_ball

# Install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# Build the workspace
$ catkin_make

# Activate the workspace (i.e. source it)
$ source devel/setup.bash

# Launch the world file in Gazebo
$        
$ 
```
