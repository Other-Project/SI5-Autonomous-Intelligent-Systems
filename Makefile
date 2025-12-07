all: sim

build_ros:
	colcon build

sim: build_ros
	. ./install/setup.sh && \
		ros2 launch turtlebot3_launch simulation.launch.py

real: build_ros
	. ./install/setup.sh && \
		ros2 launch turtlebot3_launch real.launch.py

teleop:
	export TURTLEBOT3_MODEL=burger && \
		ros2 run turtlebot3_teleop teleop_keyboard

clean:
	rm -rf build install log
