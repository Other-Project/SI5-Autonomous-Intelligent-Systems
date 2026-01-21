all: sim

build_ros:
	. .venv/bin/activate && \
		colcon build

sim: build_ros
	uv sync --extra sim
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
	export PYTHONPATH='.venv/lib/python3.12/site-packages' && \
		. .venv/bin/activate && \
		. ./install/setup.sh && \
		ros2 launch turtlebot3_launch simulation.launch.py

sim_teleop:
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
	export TURTLEBOT3_MODEL=burger && \
		ros2 run turtlebot3_teleop teleop_keyboard

real: build_ros
	export ROS_DOMAIN_ID=5 && \
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
	export PYTHONPATH='.venv/lib/python3.12/site-packages' && \
		. .venv/bin/activate && \
		. ./install/setup.sh && \
		ros2 launch turtlebot3_launch real.launch.py

deploy: build_ros
	uv sync --extra deploy
	export ROS_DOMAIN_ID=5 && \
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
	export ROS_LOG_LEVEL="rmw_cyclonedds_cpp=error" && \
	export PYTHONPATH='.venv/lib/python3.10/site-packages' && \
		. .venv/bin/activate && \
		. ./install/setup.sh && \
		ros2 launch turtlebot3_launch deploy.launch.py

teleop:
	export ROS_DOMAIN_ID=5 && \
	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
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
