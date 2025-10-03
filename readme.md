# turtlesim_reset

A custom RViz2 panel for controlling the **Turtlesim** robot in ROS 2.  
This panel allows you to move the turtle to a target position using a simple feedback controller.

## Features

- Real-time display of the turtle's position (`x`, `y`, `θ`).
- "GO!" button to start moving the turtle toward a predefined target.
- Simple proportional feedback controller for smooth motion.
- Designed as an RViz2 panel plugin, easily extendable.

## Installation

1. Make sure you have a ROS 2 workspace set up:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
