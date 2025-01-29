# F1TENTH ROS2 Wall Follower Controller

This repository contains the implementation of a wall follower controller for the F1TENTH Autonomous Racing competition. The controller was developed for use with the AutoDrive simulator and submitted via Docker Hub for qualification.

## Overview

The wall follower controller uses LIDAR data to maintain a constant distance from the walls of the track. It adjusts the steering angle and speed of the vehicle based on the distance to the nearest wall and obstacles in front of the car.

## Features

- PID control for smooth wall following
- Dynamic speed adjustment based on front distance and steering angle
- Safety measures to prevent collisions
- ROS2 integration for communication with the AutoDrive simulator

## Prerequisites

- ROS2 (tested with Foxy Fitzroy)
- AutoDrive simulator
- Docker (for containerization and submission)

## Docker Image

Here you can find the full image for the final project submission you can follow the instructions on DockerHub and use it right away:

https://hub.docker.com/r/theshaboury/mms_autonomous

## Building the Project

1. Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/TheShaboury/ROS2_Wall_Follower_PID
```
2. Build the project:

```bash
cd ~/ros2_ws
colcon build --packages-select ROS2_Wall_Follower_PID
```
3. Source the setup file:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running the controller

To run the wall follower controller:

```bash
ros2 run ROS2_Wall_Follower_PID wall_follower_node
```

## Docker submission

The controller can be containerized using Docker for submission to the F1TENTH competition. To build and push the Docker image:

```bash
docker build -t yourdockerhubusername/wall-follower-f1tenth:latest .
docker push yourdockerhubusername/wall-follower-f1tenth:latest
```

