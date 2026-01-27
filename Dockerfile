# Start from the full ROS 2 Humble Desktop image (includes RViz)
FROM osrf/ros:humble-desktop

# Install CycloneDDS and any other tools you need
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-depthai-ros ros-humble-turtlebot3-navigation2 ros-humble-turtlebot3-description

RUN apt install -y nano

RUN curl -LsSf https://astral.sh/uv/install.sh | sh

RUN git clone https://github.com/Other-Project/SI5-Autonomous-Intelligent-Systems.git

WORKDIR /SI5-Autonomous-Intelligent-Systems

# Patch repo files for ROS Humble / Python 3.10.12
RUN set -eux; \
    sed -i -E 's|export PYTHONPATH=.*$|export PYTHONPATH=.venv/lib/python3.10/site-packages|' Makefile; \
    # Set exact Python requirement in pyproject.toml
    sed -i -E 's/(requires-python\s*=\s*").*(")/\1==3.10.12\2/' pyproject.toml || true; \
    # Ensure .python-version is 3.10.12
    printf '3.10.12\n' > .python-version; \
    # Update nav2_bringup params_file to nav2_params_deployed.yaml
    sed -i -E 's/nav2_params.yaml/nav2_params_deployed.yaml/' src/turtlebot3_launch/launch/common_computer.launch.py || true

# Set environment variables permanently
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=5

# Default command
CMD ["bash"]
