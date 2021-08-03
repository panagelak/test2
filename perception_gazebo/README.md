# Assistant Gazebo Simulation

## Overview

This package is to facilitate easily starting the Gazebo simulation of the Assistant project

## Procedure to start the gazebo simulation

``` bash
# start the main gazebo simulation launch file

roslaunch assistant_gazebo assistant_gazebo.launch

# start move group for moveit

roslaunch assistant_moveit_interface assistant_moveit.launch

# start all the action/service handlers of assistant

roslaunch assistant_handlers main_assistant_handlers.launch

# start RViz

roslaunch assistant_viz rviz.launch config:=moveit

```

## Procedure without Gazebo simulation (using moveit fake controllers)

``` bash
# Launch move group + robot_description

roslaunch assistant_moveit_interface assistant_moveit_fake.launch

# start all the action/service handlers of assistant

roslaunch assistant_handlers main_assistant_handlers.launch

# start RViz

roslaunch assistant_viz rviz.launch config:=moveit

```