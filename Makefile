all: sim

build:
	colcon build

sim: build
	. install/setup.sh
	export TURTLEBOT3_MODEL=burger
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
