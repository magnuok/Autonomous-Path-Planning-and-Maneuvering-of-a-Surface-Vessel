#!/usr/bin/env python
"""
Plotting utilities for local planner.

uthor: Magnus Knædal
Date: 10.06.2020

"""

import rospy
import math
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped

def static_var(**kwargs):
    """
    This function creates decorator. Used for counting recursive calls.
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


class Plot_utils:
    """Plotting utils for rrt in rviz and pyplot.
    """
    
    ### Py plotting ###

    def plot_map(self):
        # Plot pgm map
        fig, ax = plt.subplots()
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        x_scale =  resolution * width
        y_scale = resolution * height
        org_x = self.map.info.origin.position.x
        org_y = self.map.info.origin.position.y

        imgplot = ax.imshow(np.flipud(self.grid), extent=[0 + org_x, x_scale + org_x, 0 + org_y, y_scale + org_y])
        imgplot.set_cmap('Reds')

        fig.colorbar(imgplot, ax=ax)        
        plt.xlabel(r'East $[m]$', fontsize = 12)
        plt.ylabel(r'North $[m]$', fontsize = 12)
        plt.title("Cost map", fontsize = 12)

        plt.tight_layout()
        plt.show()

    def py_plotting(self, node_list, root, goal, goal_region, path, obstacles, title, iter, cBest = None):
        
        # Plot pgm map
        fig, ax = plt.subplots()
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        x_scale =  resolution * width
        y_scale = resolution * height
        org_x = self.map.info.origin.position.x
        org_y = self.map.info.origin.position.y
        imgplot = ax.imshow(np.flipud(self.grid), extent=[0 + org_x, x_scale + org_x, 0 + org_y, y_scale + org_y])
        imgplot.set_cmap('Greys')
        #fig.colorbar(imgplot, ax=ax)

        # Plot rrt tree
        self.py_plot_root_tree(root, ax)
        # Set decorator to false again
        Plot_utils.py_plot_root_tree.block_label = False
        Plot_utils.py_plot_root_tree.tree_label = False

        # Plot path if found
        i = 1
        global_path_x = []
        global_path_y = []
        for node in path:
            global_path_x.append(node.x)
            global_path_y.append(node.y)
            if i == 1: # plot with label once
                ax.plot(node.x, node.y, 'b.', markersize = 10, label = r'Start/goal')
                i += 1
            elif i == 2: 
                ax.plot(node.x, node.y, 'g.', markersize = 10, label = r'Global waypoints')
                i += 1
            else:
                ax.plot(node.x, node.y, 'g.', markersize = 10)      
        
        ax.plot(global_path_x[-1], global_path_y[-1], 'b.', markersize = 10)

        
        # Goal region
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x_o = [goal_region * math.cos(it) for it in t]
        y_o = [goal_region * math.sin(it) for it in t]
        fx = np.array([x_o, y_o])
        px = np.array(fx[0, :] + goal.x).flatten()
        py = np.array(fx[1, :] + goal.y).flatten()        
        ax.plot(px, py, 'c', linewidth=1, label=r'Goal region')

        # Plot goal path
        ax.plot(global_path_x, global_path_y, 'g-', linewidth = 1, label = r'Path')

        # Ellipse
        if cBest and cBest != float('Inf'):
            cMin, xCenter, _, etheta = self.compute_sampling_space(root, goal)
            # Prevent math-error when they are ~equal
            if cBest < cMin:
                cBest = cMin

            a = math.sqrt(cBest ** 2 - cMin ** 2) / 2.0
            b = cBest / 2.0
            angle = math.pi / 2.0 - etheta
            cx = xCenter[0]
            cy = xCenter[1]
            t = np.arange(0, 2 * math.pi + 0.1, 0.1)
            x = [a * math.cos(it) for it in t]
            y = [b * math.sin(it) for it in t]
            R = np.array([[math.cos(angle), math.sin(angle)],
                        [-math.sin(angle), math.cos(angle)]])
            fx = R @ np.array([x, y])
            px = np.array(fx[0, :] + cx).flatten()
            py = np.array(fx[1, :] + cy).flatten()

            ax.plot(px, py, 'm-', linewidth=1, label=r'Sampling ellipse')

        # Obstacles
        if len(obstacles) != 0:
            i = 1
            t = np.arange(0, 2 * math.pi + 0.1, 0.1)
            for obstacle in obstacles:
                ox = obstacle[0]
                oy = obstacle[1]
                size = obstacle[2]
                x = [size * math.cos(it) for it in t]
                y = [size * math.sin(it) for it in t]
                fx = np.array([x, y])
                px = np.array(fx[0, :] + ox).flatten()
                py = np.array(fx[1, :] + oy).flatten()

                if i == 1: # plot with label once
                    ax.plot(px, py, 'r-', linewidth=1, label=r'Obstacle region')
                    i += 1
                else:
                    ax.plot(px, py, 'r-', linewidth=1)

        plt.xlabel(r'East $[m]$')
        plt.ylabel(r'North $[m]$')
        plt.title(title, fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right', fontsize = 'xx-small')
        plt.tight_layout()
        
        s = "/home/revolt/sim/" + str(iter) + ".pdf"
        plt.savefig(s)

        #plt.show()

    @static_var(tree_label = False, block_label = False)
    def py_plot_root_tree(self, node, ax):
        for child in node.children:
            path_x = [node.x, child.x]
            path_y = [node.y, child.y]

            # Blocked
            if child.cost == float('Inf') or node.cost == float('Inf'):
                # plot with label once                               
                if Plot_utils.py_plot_root_tree.block_label == False:
                    Plot_utils.py_plot_root_tree.block_label = True
                    ax.plot(path_x, path_y, marker = ".", color = "red", linewidth = 1, label=r'Blocked branches')
                else:
                    ax.plot(path_x, path_y, marker = ".", color = "red", linewidth = 1)
                
            # Regular
            else:         
                # plot with label once               
                if Plot_utils.py_plot_root_tree.tree_label == False:
                    Plot_utils.py_plot_root_tree.tree_label = True
                    ax.plot(path_x, path_y, marker = ".", color = "orange", linewidth = 1, label=r'RRT tree')
                else:
                    ax.plot(path_x, path_y, marker = ".", color = "orange", linewidth = 1)

            self.py_plot_root_tree(child, ax)

    ### RViz plotting ###

    def draw_graph(self, root, goal, cBest, current_path):
        """
        Main plotting function.

        """
        # Ellipse
        if cBest != float('Inf'):
            cMin, xCenter, _, etheta = self.compute_sampling_space(root, goal)
            self.plot_ellipse(xCenter, cBest, cMin, etheta)

        # Tree edges between nodes
        #self.plot_edges(self.node_list)
        self.tree_marker = MarkerArray()
        self.rviz_create_root_tree(root, 1)
        self.pub_edges.publish(self.tree_marker)

        # Node/vertex itself
        self.plot_nodes(self.node_list)
        # Plot obstacles
        self.plot_obstacles(self.obstacle_list)
        # Plot root/vessel
        self.plot_root(self.root)
        # Plot start and goal
        self.plot_start_and_goal(self.start, self.goal_node)
        
        self.plot_goal_region(self.goal_node, self.goal_region)
        # Plot blocked nodes
        self.plot_blocked_nodes(self.node_list, self.visited_set)
        # Plot plot_blocked_edges
        self.plot_blocked_edges(self.node_list)

        # Plot current path
        self.plot_path(current_path)

    def rviz_create_root_tree(self, node, i):
        for child in node.children:
            edge = self.create_edge(node, child, i)                
            self.tree_marker.markers.append(edge)
            i += 1
            self.rviz_create_root_tree(child, i)
    
    def create_edge(self, node, child, i):
        # edge between nodes
        path = Marker()
        path.header.frame_id = "map"
        path.header.stamp = rospy.get_rostime()
        path.ns = "markers"
        path.id = i
        path.type = path.LINE_STRIP
        path.action = path.ADD
        path.scale.x = self.rviz_tuning_plt
        path.color.a = 1.0

        path.color.r = 1.0
        path.color.g = 0.7
        path.color.b = 0.0

        path.lifetime = rospy.Duration()
        path.pose.orientation.w = 1.0

        p1 = Point()
        p1.x = node.x
        p1.y = node.y
        p1.z = 0.02
        path.points.append(p1)

        p2 = Point()
        p2.x = child.x
        p2.y = child.y
        p2.z = 0.02
        path.points.append(p2)

        return path

    def plot_edges(self, node_list):
        """
        Plot edges between nodes in the rrt tree.
        """
        tree = MarkerArray()
        id = 1
        for node in self.node_list:
            if node.parent:
                # edge between nodes
                path = Marker()
                path.header.frame_id = "map"
                path.header.stamp = rospy.get_rostime()
                path.ns = "markers"
                path.id = id
                id += 1
                path.type = path.LINE_STRIP
                path.action = path.ADD
                path.scale.x = self.rviz_tuning_plt
                path.color.a = 1.0

                path.color.r = 1.0
                path.color.g = 0.7
                path.color.b = 0.0

                path.lifetime = rospy.Duration()
                path.pose.orientation.w = 1.0

                p1 = Point()
                p1.x = node.parent.x
                p1.y = node.parent.y
                p1.z = 0.02
                path.points.append(p1)

                p2 = Point()
                p2.x = node.x
                p2.y = node.y
                p2.z = 0.02
                path.points.append(p2)
    
                tree.markers.append(path)

        self.pub_edges.publish(tree)

    def plot_nodes(self, node_list):
        """
        Plots the nodes in the rrt tree
        """
        points = Marker()
        #visualizations  points and lines..
        points.header.frame_id = "map"
        points.header.stamp = rospy.get_rostime()
        points.ns = "markers"
        points.id = 0
        points.type = points.POINTS
        points.action = points.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 2*self.rviz_tuning_plt
        points.scale.y = 2*self.rviz_tuning_plt
        points.color.r = 0.0
        points.color.g = 1.0
        points.color.b = 0.0
        points.color.a = 1.0
        points.lifetime = rospy.Duration()

        for node in node_list:
            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.01
            points.points.append(p1)
        
        self.pub_nodes.publish(points)
    
    def plot_blocked_nodes(self, node_list, visited_set):
        """
        Plots nodes blocked by obstacles, and by planning
        """
        points = Marker()
        #visualizations  points and lines..
        points.header.frame_id = "map"
        points.header.stamp = rospy.get_rostime()
        points.ns = "markers"
        points.id = 0
        points.type = points.POINTS
        points.action = points.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 2*self.rviz_tuning_plt
        points.scale.y = 2*self.rviz_tuning_plt
        points.color.r = 1.0
        points.color.g = 0.0
        points.color.b = 0.0
        points.color.a = 1.0
        points.lifetime = rospy.Duration()

        # Nodes blocked by obstacles
        for node in node_list:
            if node.cost == float('Inf'):
                p1 = Point()
                p1.x = node.x
                p1.y = node.y
                p1.z = 0.011
                points.points.append(p1)

        # Nodes blocked by planner
        for node in visited_set:
            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.03
            points.points.append(p1)
        
        self.pub_blocked_nodes.publish(points)
    
    def plot_blocked_edges(self, node_list):
        """
        Plot blocked branches in the rrt tree.
        """
        tree = MarkerArray()
        id = 1
        for node in self.node_list:
            if node.parent and node.parent.cost == float('Inf'):
                # edge between nodes
                path = Marker()
                path.header.frame_id = "map"
                path.header.stamp = rospy.get_rostime()
                path.ns = "markers"
                path.id = id
                id += 1
                path.type = path.LINE_STRIP
                path.action = path.ADD
                path.scale.x = self.rviz_tuning_plt
                path.color.a = 1.0

                path.color.r = 1.0
                path.color.g = 0.0
                path.color.b = 0.0

                path.lifetime = rospy.Duration()
                path.pose.orientation.w = 1.0

                p1 = Point()
                p1.x = node.parent.x
                p1.y = node.parent.y
                p1.z = 0.03
                path.points.append(p1)

                p2 = Point()
                p2.x = node.x
                p2.y = node.y
                p2.z = 0.03
                path.points.append(p2)
    
                tree.markers.append(path)

        self.pub_blocked_edges.publish(tree)  

    def plot_path(self, current_path):
        """
        Plots the current path chosen.
        """
        full_path = current_path.copy()
        full_path.insert(0, self.root)

        path = Marker()
        id = 1

        # edge between nodes
        path = Marker()
        path.header.frame_id = "map"
        path.header.stamp = rospy.get_rostime()
        path.ns = "markers"
        path.id = 1
        path.type = path.LINE_STRIP
        path.action = path.ADD
        path.scale.x = self.rviz_tuning_plt
        path.color.a = 1.0
        path.color.r = 0.0
        path.color.g = 1.0
        path.color.b = 0.0

        path.lifetime = rospy.Duration()
        path.pose.orientation.w = 1.0

        for node in full_path:
            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.03
            path.points.append(p1)

        self.pub_path.publish(path)

    def plot_ellipse(self, xCenter, cBest, cMin, etheta):

        # Prevent math-error when they are ~equal
        if cBest < cMin:
            cBest = cMin

        a = math.sqrt(cBest ** 2 - cMin ** 2) / 2.0
        b = cBest / 2.0
        angle = math.pi / 2.0 - etheta
        cx = xCenter[0]
        cy = xCenter[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        R = np.array([[math.cos(angle), math.sin(angle)],
                      [-math.sin(angle), math.cos(angle)]])
        fx = R @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()

        ellipse = Marker()
        ellipse.header.frame_id = "map"
        ellipse.header.stamp = rospy.get_rostime()
        ellipse.ns = "markers"
        ellipse.id = 1
        ellipse.type = ellipse.LINE_STRIP
        ellipse.action = ellipse.ADD

        ellipse.scale.x = 0.1 
        #ellipse.scale.y = 0.03
        ellipse.color.r = 0.0
        ellipse.color.g = 0.3
        ellipse.color.b = 7.0
        ellipse.color.a = 1.0
        ellipse.lifetime = rospy.Duration()
        ellipse.pose.orientation.w = 1.0

        for (x, y) in zip(px, py):
            p1 = Point()
            p1.x = x
            p1.y = y
            p1.z = 0
            ellipse.points.append(p1)
        
        self.pub_ellipse.publish(ellipse)
    
    def plot_obstacles(self, obstacles):
        """
        Plots obstacle circles.

        """
        # First delete all obstacles
        obst_array = MarkerArray()
        obst = Marker()
        obst.header.frame_id = "map"
        obst.header.stamp = rospy.get_rostime()
        obst.ns = "markers"
        obst.id = 0
        obst.type = obst.LINE_STRIP
        obst.action = obst.DELETEALL
        obst_array.markers.append(obst)
        self.pub_obst.publish(obst_array)

        # Create new obstacles
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)

        obst_array = MarkerArray()
        id = 1

        for obstacle in obstacles:
            ox = obstacle[0]
            oy = obstacle[1]
            size = obstacle[2]
            obst = Marker()
            obst.header.frame_id = "map"
            obst.header.stamp = rospy.get_rostime()
            obst.ns = "markers"
            obst.id = id
            id += 1
            obst.type = obst.LINE_STRIP
            obst.action = obst.ADD

            obst.scale.x = self.rviz_tuning_plt
            
            obst.color.r = 1.0
            obst.color.g = 0.0
            obst.color.b = 0.0
            obst.color.a = 1.0
            obst.lifetime = rospy.Duration()
            obst.pose.orientation.w = 1.0

            x = [size * math.cos(it) for it in t]
            y = [size * math.sin(it) for it in t]
            fx = np.array([x, y])
            px = np.array(fx[0, :] + ox).flatten()
            py = np.array(fx[1, :] + oy).flatten()
            for (x, y) in zip(px, py):
                p1 = Point()
                p1.x = x
                p1.y = y
                p1.z = 0
                obst.points.append(p1)
            
            obst_array.markers.append(obst)

        self.pub_obst.publish(obst_array)

    def plot_goal_region(self, goal, goal_region):
        """
        Plots obstacle circles.

        """

        # Create new obstacles
        id = 1

        obst = Marker()
        obst.header.frame_id = "map"
        obst.header.stamp = rospy.get_rostime()
        obst.ns = "markers"
        obst.id = id
        id += 1
        obst.type = obst.LINE_STRIP
        obst.action = obst.ADD

        obst.scale.x = self.rviz_tuning_plt
        
        obst.color.r = 0.0
        obst.color.g = 1.0
        obst.color.b = 0.0
        obst.color.a = 1.0
        obst.lifetime = rospy.Duration()
        obst.pose.orientation.w = 1.0

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [goal_region * math.cos(it) for it in t]
        y = [goal_region * math.sin(it) for it in t]
        fx = np.array([x, y])
        px = np.array(fx[0, :] + goal.x).flatten()
        py = np.array(fx[1, :] + goal.y).flatten()
        for (x, y) in zip(px, py):
            p1 = Point()
            p1.x = x
            p1.y = y
            p1.z = 0
            obst.points.append(p1)
        
        self.pub_goal_region.publish(obst)
    
    def plot_start_and_goal(self, start_node, goal_node):
        """
        Plots start and goal as a pose.
        """
        start = Marker()
        #visualizations  points and lines..
        start.header.frame_id = "map"
        start.header.stamp = rospy.get_rostime()
        start.ns = "markers"
        start.id = 1
        start.type = start.ARROW
        start.action = start.ADD
        
        start.scale.x = 10*self.rviz_tuning_plt
        start.scale.y = 2*self.rviz_tuning_plt
        start.scale.z = 2*self.rviz_tuning_plt
        start.color.r = 0.0
        start.color.g = 1.0
        start.color.b = 0.0
        start.color.a = 1.0
        # A value of ros::Duration() means never to auto-delete.
        start.lifetime = rospy.Duration()
        # Add x,y,z position to pose
        start.pose.position.x = start_node.x
        start.pose.position.y = start_node.y
        start.pose.position.z = 0
        # add quaternion to pose
        quat = self.euler_to_quaternion(0, 0, start_node.alpha)
        start.pose.orientation.x = quat[0]
        start.pose.orientation.y = quat[1]
        start.pose.orientation.z = quat[2]
        start.pose.orientation.w = quat[3]
        self.pub_start_goal.publish(start)
        
        goal = start
        goal.id = 2
        goal.color.b = 1
        # Add x,y,z position to pose
        goal.pose.position.x = goal_node.x
        goal.pose.position.y = goal_node.y
        goal.pose.position.z = 0
        # add quaternion to pose
        quat = self.euler_to_quaternion(0, 0, goal_node.alpha)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        self.pub_start_goal.publish(goal)

    def plot_root(self, root):
        """
        Plots the root of the tree as as pose (Next WP.)
        """
        vessel = Marker()
        #visualizations  points and lines..
        vessel.header.frame_id = "map"
        vessel.header.stamp = rospy.get_rostime()
        vessel.ns = "markers"
        vessel.id = 1
        vessel.type = vessel.ARROW
        vessel.action = vessel.ADD
        
        vessel.scale.x = 10*self.rviz_tuning_plt
        vessel.scale.y = 2*self.rviz_tuning_plt
        vessel.scale.z = 2*self.rviz_tuning_plt
        vessel.color.r = 0.0
        vessel.color.g = 0.0
        vessel.color.b = 1.0
        vessel.color.a = 1.0
        # A value of ros.Duration() means never to auto-delete.
        vessel.lifetime = rospy.Duration()
        # Add x,y,z position to pose
        vessel.pose.position.x = root.x
        vessel.pose.position.y = root.y
        vessel.pose.position.z = 0
        # add quaternion to pose
        quat = self.euler_to_quaternion(0, 0, root.alpha)
        vessel.pose.orientation.x = quat[0]
        vessel.pose.orientation.y = quat[1]
        vessel.pose.orientation.z = quat[2]
        vessel.pose.orientation.w = quat[3]
        self.pub_root.publish(vessel)
