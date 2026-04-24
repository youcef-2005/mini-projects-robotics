# Mini-Project 1: ROS 2 Publisher/Subscriber Communication

## Overview
This project demonstrates a basic asynchronous communication system between two nodes using the ROS 2 Python API (`rclpy`).

## Objectives
* Implement a **Publisher** node to broadcast data on a specific topic.
* Implement a **Subscriber** node to listen and process incoming data.
* Understand the use of standard message types (`std_msgs`).

## Project Structure
* `publisher.py`: Generates and sends a string message every second.
* `subscriber.py`: Subscribes to the topic and logs the received messages.

## How it works
The communication relies on the **Topic** named `mini_project_topic`. The publisher acts as a simulated sensor data stream, while the subscriber mimics a monitoring or processing unit.
