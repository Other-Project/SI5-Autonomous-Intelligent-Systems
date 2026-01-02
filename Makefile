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

package:
	read -p "Name: " name; \
	read -p "Description: " desc; \
	. /opt/ros/*/setup.sh && \
	ros2 pkg create --build-type ament_python \
		--maintainer-email "evan.galli@etu.univ-cotedazur.fr" --maintainer-name "Evan Galli" --license "MIT" \
		--destination-directory "./src/" --node-name "node" --description "$${desc}" "$${name}"

clean:
	rm -rf build install log

mass_shooting:
	ps aux | grep gz | grep -v grep | awk '{print $$2}' | xargs kill -9
