# Turtlebot3

<p align=center>
    <img src=".assets/turtlebot3.png" width="20%" alt="Turtlebot3" /><br/>
    <a href="https://releases.ubuntu.com/noble/">
        <img alt="Ubuntu 24.04" src="https://img.shields.io/badge/-UBUNTU%2024%2E04-orange?style=flat-square&logo=ubuntu&logoColor=white" />
    </a>
    <a href="https://docs.ros.org/en/jazzy/index.html">
        <img alt="ROS Jazzy" src="https://img.shields.io/badge/-ROS%20JAZZY-blue?style=flat-square&logo=ros" />
    </a><br />
    <span>Project realized by <a href="https://github.com/06Games">Evan Galli</a>, <a href="https://github.com/LubratJilian">Jilian Lubrat</a>, <a href="https://github.com/eliotmnrt">Eliot Menoret</a> and <a href="https://github.com/mantoniu">Antoine-Marie Michelozzi</a>
    <br/>as part of the <b>Autonomous Intelligent Systems</b> and <b>Edge computing & Embedded AI</b> courses.</span>
</p>

## Features

* Person recognition: The bot exclusively recognizes and obeys commands from its "master".
* Hand-gesture control: Use hand signals to pair with the bot and direct its actions.
* Person following: The bot actively tracks and follows the "master" until signaled to stop.

## Hardware

* Turtlebot3
* Luxonis OAK-D Pro 3d camera

## Installation

Install [ROS Jazzy](https://docs.ros.org/en/jazzy/index.html) and [uv](https://docs.astral.sh/uv/) to run this project

Then install the following packages:

```sh
sudo apt install ros-jazzy-depthai-ros ros-jazzy-turtlebot3-navigation2
```

Setup the Luxonis OAK-D Pro camera by following the instructions [here](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide/).

## Usage

* `make sim` to start the project in simulation mode
* `make deploy` to start the project in a deployed environment
* `make real` to start the project on a computer connected to the Turtlebot3
* `make teleop` to manually control the robot with keyboard inputs
* `make clean` to clean the workspace
* `make mass_shooting` to kill all remaining simulation processes
