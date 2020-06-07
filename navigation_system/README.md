navigation\_system
===

This repo includes ros packages for a navigation system for a surface vessel, including a local and a global planner. The planners builds on occupancy grid maps. Global planner uses a A* search on a Voronoi graph. Local planner uses a real-time extension of RRT* to handle dynamic changes in environment.

# Installation
Have a look at the [INSTALL.md](INSTALL.md) file

# Packages
* global\_planner
* local\_planner
* tuw\_msgs
* tuw\_multi\_robot\_rviz
* tuw\_voronoi\_graph
* dyn\_config

# System overview

## global\_planner
Contains implementation of global planner. Lauch file in launching pacakge.

## local\_planner
Contains implementation of local planner. Lauch file in launching pacakge.

## tuw\_voronoi\_graph
This package includes a voronoi-graph-generator a dxf-to-graph-node and a segment-to-graph node for creating Voronoi search graphs.

The _voronoi-graph-generator-node_ receives a pixel map ([occupancy\_grid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html)) and converts it into a voronoi graph describing the original map. This graph is automatically generated or loaded from a cache folder if saved. Additionally the node can load specific graphs saved in a folder.

The _voronoi-dxf-to-graph_ takes a dxf file as input containing the scaled and transformed map and any number of lines arcs and circles. These lines arcs and circles are converted to a graph and saved to a specific location.

The _voronoi-segment-to-graph-node_ takes a segment file with predefined segments as input and converts it to a graph, which is published afterwards.

The _graph-generator-node_ receives a grid\_map like the one published from voronoi-map-generator-node and converts it into a graph message.

## tuw\_multi\_robot\_rviz
Presents rviz plugins to set goal positions for the planner and a tool to visualize generated graphs. 

<img src="tuw_multi_robot/res/rviz_plugin.png" alt="rviz plugin" width="300px"/>

## dyn\_config
Contains dynamic reconfig files for rqt GUI.

# References
http://wiki.ros.org/tuw_multi_robot

