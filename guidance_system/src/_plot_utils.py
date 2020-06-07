#!/usr/bin/env python
"""
Plot utils for guidance system.

Author: Magnus Knaedal
Date: 10.06.2020

"""
import rospy
import math
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    @static_var(path_id=0)
    def plot_path(self, pg_output, path, i):
        path.header.frame_id = "map"
        path.header.stamp = rospy.get_rostime()
        path.header.seq = 0
                        
        for eta_d in pg_output.eta_d:
            pose = PoseStamped()
            #pose.header.seq = Plot_utils.plot_path.path_id
            pose.header.seq = i
            #Plot_utils.plot_path.path_id += 1
            pose.header.stamp = rospy.get_rostime()
            pose.header.frame_id = "map"

            pose.pose.position.x = eta_d[0]
            pose.pose.position.y = eta_d[1]
            pose.pose.position.z = 0
            quat = self.euler_to_quaternion(0, 0, eta_d[2])
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            path.poses.append(pose)
        
        self.pub_path.publish(path)

    def plot_s_dynamic(self, eta_d):
        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.seq = 0
        pose.header.stamp = rospy.get_rostime()

        # Add x,y,z position to pose
        pose.pose.position.x = eta_d[0]
        pose.pose.position.y = eta_d[1]
        pose.pose.position.z = 0
        # add quaternion to pose
        quat = self.euler_to_quaternion(0, 0, eta_d[2])
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.pub_s_dyn.publish(pose)

    @staticmethod
    def plotting(plotlist):
        
        for plot_object in plotlist:

            WP_current = plot_object.WP_current
            WP_next = plot_object.WP_next
            CP_opt = plot_object.CP_opt
            zeta = plot_object.zeta
            pg_output = plot_object.pg_output
            i = plot_object.i
            s = plot_object.s
            
            # Interactive mode.
            plt.ion()

            # Walls:
            norm = math.hypot(WP_next[0] - WP_current[0], WP_next[1] - WP_current[1])
            u = np.divide(WP_next - WP_current, norm) 
            z = zeta*u; # distance away
            rot = np.array([-z[1], z[0]]); # counter clockwise rotation

            ## With ctrl points
            plt.figure(1)

            plt.grid(True)
            plt.axis('equal')
            plt.plot(CP_opt[:,0], CP_opt[:,1], 'k.-', markersize = 7, label='Control polygon')
            plt.plot(WP_current[0], WP_current[1], 'ko', markersize = 12, fillstyle='none')
            plt.plot(WP_next[0], WP_next[1], 'ko', markersize = 12, fillstyle='none', label='Waypoints')
            plt.plot(pg_output.p_d[:,0], pg_output.p_d[:,1], Color = 'r', LineWidth=1.5, label=r'$B(\theta)$')
            p1 = np.subtract(np.array([WP_current[0], WP_next[0]]), rot[0])
            p2 = np.subtract(np.array([WP_current[1], WP_next[1]]), rot[1])
            plt.plot(p1, p2,'--b', label='Walls')
            p1 = np.add(np.array([WP_current[0], WP_next[0]]), rot[0])
            p2 = np.add(np.array([WP_current[1], WP_next[1]]), rot[1]) 
            plt.plot(p1, p2,'--b')
            if i == 0:
                plt.legend(shadow = True)
            
            plt.tight_layout()
            ## Without ctrl points
            # plt.figure(2)
            # plt.grid(True)
            # plt.axis('equal')
            # plt.plot(pg_output.p_d[:,0], pg_output.p_d[:,1], 'r', LineWidth=1.5)
            # p1 = np.subtract(np.array([WP_current[0], WP_next[0]]), rot[0])
            # p2 = np.subtract(np.array([WP_current[1], WP_next[1]]), rot[1])
            # plt.plot(p1, p2,'--b')
            # p1 = np.add(np.array([WP_current[0], WP_next[0]]), rot[0])
            # p2 = np.add(np.array([WP_current[1], WP_next[1]]), rot[1]) 
            # plt.plot(p1, p2,'--b')

            # Direction and curvature for each type
            plt.figure(3)
            plt.subplot(311)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot( s , pg_output.psi_deg, Color = 'r', LineWidth = 1.5)
            plt.ylabel(r'$\psi(s) \: [deg]$', fontsize = 12)
            plt.title(r'Path Direction', fontsize = 12)
            #  Curvature
            plt.subplot(312)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot( s , pg_output.K, Color = 'r', LineWidth = 1.5)
            plt.ylabel(r'$\kappa(s) \: [m^{-1}]$', fontsize = 12)
            plt.title(r'Path Curvature', fontsize = 12)
            #  Rate of change in curvature
            plt.subplot(313)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot( s , pg_output.dot_K, Color = 'r', LineWidth = 1.5)
            plt.ylabel(r'$\tau(s) \: [(m/s)^{-1}]$', fontsize = 12)
            plt.title(r'Rate of Change in Path Curvature', fontsize = 12)

            #plt.xlabel(r'$s = \theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', fontsize = 12)
            plt.tight_layout(pad=0.0)
            
            # Speed profile:
            plt.figure(5)
            
            plt.subplot(311)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot(s, pg_output.v, Color = 'r', LineWidth = 1.5)
            plt.ylabel(r'$v(s,t) [m/s]$', fontsize = 12)
            plt.title(r'The Speed Profile and its Respective Derivatives', fontsize = 12)

            plt.subplot(312)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot(s, pg_output.dtheta_v, Color = 'r', LineWidth = 1.5)
            plt.ylabel(r'$v^{s}(s,t) \: [m/s]$', fontsize = 12)
            
            plt.subplot(313)
            plt.xticks(np.arange(0, 100, 1))
            plt.grid(True)
            plt.plot(s, pg_output.dt_v, 'r', LineWidth = 1.5)
            plt.xlabel(r'$s = \theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', fontsize = 12)
            plt.ylabel(r'$v^{t}(s,t) \: [m/s^2]$', fontsize = 12)
            
            plt.tight_layout(pad=0.0)

            # Derivatives
            plt.figure(4)
            # first derivative
            plt.subplot(311)
            plt.xticks(np.arange(0, 100, 1))
            plt.plot(s, pg_output.dot_p_d[:,0], 'r', LineWidth = 1)
            plt.plot(s, pg_output.dot_p_d[:,1], 'b', LineWidth = 1)
            plt.ylabel(r'$[deg]$', fontsize = 12)
            plt.title(r'First derivative', fontsize = 12)
            #  second derivative
            plt.subplot(312)
            plt.xticks(np.arange(0, 100, 1))
            plt.plot(s, pg_output.ddot_p_d[:,0], Color = 'r', LineWidth = 1.5)
            plt.plot(s, pg_output.ddot_p_d[:,1], Color = 'b', LineWidth = 1.5)
            plt.ylabel(r'$[deg]$', fontsize = 12)
            plt.title(r'Second derivative', fontsize = 12)
            #  third derivative
            plt.subplot(313)
            plt.xticks(np.arange(0, 100, 1))
            plt.plot(s, pg_output.dddot_p_d[:,0], Color = 'r', LineWidth = 1.5)
            plt.plot(s, pg_output.dddot_p_d[:,1], Color = 'b', LineWidth = 1.5)
            plt.xlabel(r'$s = \theta+i-1, \: \theta \in [0,1], \: i \in \mathcal{I}^m$', fontsize = 12)
            plt.ylabel(r'$[deg]$', fontsize = 12)
            plt.title(r'Third derivative', fontsize = 12)
            plt.tight_layout(pad=0.0) 

        # Starts the mock-up event-loop with pause such
        # that it has time to run at least once and produce the figure in completeness.
        # TODO: Should call pause repeatedly in main thread afterwards to not let the window freeze.
        plt.draw()
        plt.pause(400)
        
    def draw_graph(self, root, goal, cBest, rnd=None):

        if rnd is not None:
            self.plot_point([(rnd.x, rnd.y)]) #TODO: Change colour?

        if cBest != float('Inf'):
            cMin, xCenter, _, etheta = self.compute_sampling_space(root, goal)
            self.plot_ellipse(xCenter, cBest, cMin, etheta)

        # Tree.Edge between nodes
        self.plot_paths(self.node_list)
        # Node/vertex itself
        self.plot_nodes(self.node_list)
        # Plot obstacles
        self.plot_obstacles(self.obstacle_list)

        self.plot_start_and_goal(root, self.goal_node)

        self.plot_blocked_nodes()

    def plot_paths(self, node_list):
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
                path.scale.x = 0.03
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

        self.pub_tree.publish(tree)

    def plot_paths_from_root(self, root):
        tree = MarkerArray()
        node_id = 1
        
        tree, node_id = self.get_paths(root, tree, node_id)

        self.pub_root_tree.publish(tree)

    def get_paths(self, node, tree, node_id):
        
        if len(node.children) is 0:
            edge, node_id = self.create_edge(node, node_id)
            tree.markers.append(edge)
            return tree, node_id
        else:
            for child in node.children:
                edge, node_id = self.create_edge(child, node_id)
                tree.markers.append(edge)
                tree, node_id = self.get_paths(child, tree, node_id)
            return tree, node_id

    def create_edge(self, node, node_id):
        # edge between nodes
        edge = Marker()
        edge.header.frame_id = "map"
        edge.header.stamp = rospy.get_rostime()
        edge.ns = "markers"
        edge.id = node_id
        node_id += 1
        edge.type = edge.LINE_STRIP
        edge.action = edge.ADD
        edge.scale.x = 0.03
        edge.color.a = 1.0

        edge.color.r = 1.0
        edge.color.g = 0.7
        edge.color.b = 1.0

        edge.lifetime = rospy.Duration()
        edge.pose.orientation.w = 1.0
        
        p1 = Point()
        p1.x = node.parent.x
        p1.y = node.parent.y
        p1.z = 0.02
        edge.points.append(p1)

        p2 = Point()
        p2.x = node.x
        p2.y = node.y
        p2.z = 0.02
        edge.points.append(p2)

        return edge, node_id

    def plot_final_path(self, current_path):
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
        path.scale.x = 0.03
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

        self.pub_final_path.publish(path)

    def plot_nodes(self, node_list):
        points = Marker()
        #visualizations  points and lines..
        points.header.frame_id = "map"
        points.header.stamp = rospy.get_rostime()
        points.ns = "markers"
        points.id = 0
        points.type = points.POINTS
        points.action = points.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.05 
        points.scale.y = 0.05
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
        
        self.pub.publish(points)
    
    def plot_point(self, node_list):
        points = Marker()
        #visualizations  points and lines..
        points.header.frame_id = "map"
        points.header.stamp = rospy.get_rostime()
        points.ns = "markers"
        points.id = 0
        points.type = points.POINTS
        points.action = points.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.1 
        points.scale.y = 0.1
        points.color.r = 1.0
        points.color.g = 0.0
        points.color.b = 0.0
        points.color.a = 1.0
        points.lifetime = rospy.Duration()

        for node in node_list:
            p1 = Point()
            p1.x = node[0]
            p1.y = node[1]
            p1.z = 0.1
            points.points.append(p1)
        
        self.pub.publish(points)

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
        fx = R.dot(np.array([x, y]))
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()

        ellipse = Marker()
        ellipse.header.frame_id = "map"
        ellipse.header.stamp = rospy.get_rostime()
        ellipse.ns = "markers"
        ellipse.id = 1
        ellipse.type = ellipse.LINE_STRIP
        ellipse.action = ellipse.ADD

        ellipse.scale.x = 0.03 
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
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)

        obst_array = MarkerArray()
        id = 1

        for (ox, oy, size) in obstacles:
            obst = Marker()
            obst.header.frame_id = "map"
            obst.header.stamp = rospy.get_rostime()
            obst.ns = "markers"
            obst.id = id
            id += 1
            obst.type = obst.LINE_STRIP
            obst.action = obst.ADD

            obst.scale.x = 0.03 
            #ellipse.scale.y = 0.03
            obst.color.r = 0.0
            obst.color.g = 0.7
            obst.color.b = 3.0
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
    
    def plot_grid_cells(self, cells):
        grid_cells = Marker()
        #visualizations  points and lines..
        grid_cells.header.frame_id = "map"
        grid_cells.header.stamp = rospy.get_rostime()
        grid_cells.ns = "markers"
        grid_cells.id = 0
        grid_cells.type = grid_cells.POINTS
        grid_cells.action = grid_cells.ADD
        grid_cells.pose.orientation.w = 1.0
        grid_cells.scale.x = 0.05 
        grid_cells.scale.y = 0.05
        grid_cells.color.r = 0.5
        grid_cells.color.g = 0.0
        grid_cells.color.b = 1.0
        grid_cells.color.a = 1.0
        grid_cells.lifetime = rospy.Duration()

        for cell in cells:
            p1 = Point()
            p1.x = cell[0]
            p1.y = cell[1]
            p1.z = 0.1
            grid_cells.points.append(p1)
        
        self.pub_test.publish(grid_cells)

    def plot_start_and_goal(self, start_node, goal_node):
        start = Marker()
        #visualizations  points and lines..
        start.header.frame_id = "map"
        start.header.stamp = rospy.get_rostime()
        start.ns = "markers"
        start.id = 1
        start.type = start.ARROW
        start.action = start.ADD
        
        start.scale.x = 0.5
        start.scale.y = 0.05
        start.scale.z = 0.05
        start.color.r = 0.0
        start.color.g = 1.0
        start.color.b = 0.0
        start.color.a = 1.0
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

    def plot_blocked_nodes(self):
        points = Marker()
        #visualizations  points and lines..
        points.header.frame_id = "map"
        points.header.stamp = rospy.get_rostime()
        points.ns = "markers"
        points.id = 0
        points.type = points.POINTS
        points.action = points.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.05 
        points.scale.y = 0.05
        points.color.r = 1.0
        points.color.g = 0.0
        points.color.b = 0.0
        points.color.a = 1.0
        points.lifetime = rospy.Duration()

        for node in self.node_list:
            if node.cost == float('Inf'):
                p1 = Point()
                p1.x = node.x
                p1.y = node.y
                p1.z = 0.011
                points.points.append(p1)

        for node in self.visited_set:
            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.02
            points.points.append(p1)
        
        self.pub_blocked_nodes.publish(points)
    