all: sim

build_ros:
	colcon build

sim: build_ros
	. ./install/setup.sh && \
		ros2 launch turtlebot3_launch simulation.launch.py

real: build_ros
	. ./install/setup.sh && \
		ros2 launch turtlebot3_launch real.launch.py

clean:
	rm -rf build install log
