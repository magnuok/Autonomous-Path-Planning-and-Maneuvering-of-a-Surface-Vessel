#!/usr/bin/env python3
"""
Path planning Sample Code with RT-RRT*

uthor: Magnus Knædal
Date: 10.06.2020

"""
import rosbag
import rospy
import math
import numpy as np
from std_msgs.msg import String
from tuw_multi_robot_msgs.msg import Graph
from tuw_multi_robot_msgs.msg import Vertex
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import ast
from dynamic_reconfigure.server import Server
from dyn_config.cfg import GlobalTuningConfig
import matplotlib.image as mpimg
from matplotlib.markers import MarkerStyle

class GlobalPlanner:

    class Node:
        """
        Class for representing nodes in voronoi roadmap.
        """            

        def __init__(self, id, position, neighbours):
            self.id         = id
            self.position   = position
            self.neighbours = neighbours # list of neighbour indices
            self.parent     = None
            self.f          = 0
            self.g          = 0
            self.h          = 0

        def __eq__(self, other):
            """Compare two nodes

            Arguments:
                other {[type]} -- [description]

            Returns:
                [type] -- [description]
            """            
            return self.position == other.position

        def __lt__(self, other):
            """Sort list of nodes by cost

            Arguments:
                other {[type]} -- [description]

            Returns:
                [type] -- [description]
            """            
            return self.f < other.f

        def __repr__(self):
            """Print format of node

            Returns:
                [type] -- [description]
            """            
            return ('{0}, {1}, {2}, {3} \n'.format(self.id, self.position, self.f, self.neighbours))

    def __init__(self):
        """
        Init Global planner.

        :param:
            start_node: start
            goal_node: goal
            zeta: clearence threshold for pruning
            alpha_max: angle threshold for pruning (degrees)
        """

        ### Parameters ###
        x_vessel = 152.395
        y_vessel = 1128.154

        self.start_node = self.Node(-1, [x_vessel, y_vessel], [])
        self.goal_node = self.Node(-1, [x_vessel+27, y_vessel], [])
        self.zeta = 2
        self.alpha_max = 10
        self.start_angle = 0
        self.goal_angle = -math.pi/2
        self.occupancy_thres = 30

        ### Ros node, pubs and subs ### 
        rospy.init_node('global_planner', anonymous=True)
        self.pub_plot_nodes = rospy.Publisher('nav_syst/global/plotting/waypoints', Marker, queue_size = 10)
        self.pub_plot_path = rospy.Publisher('nav_syst/global/plotting/path', Marker, queue_size = 10)

        self.pub_global_path = rospy.Publisher('nav_syst/global/path', Path, queue_size = 10)

        rospy.Subscriber('/rviz/global_start', PoseWithCovarianceStamped, self.callback_rviz_start)
        rospy.Subscriber('/rviz/global_goal', PoseStamped, self.callback_rviz_goal)
        rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, self.callback_map)
        
        ### Init map ###
        self.map = None
        while self.map is None:
            continue
        self.grid = np.reshape(np.array(self.map.data), (self.map.info.width, self.map.info.height) )
        self.roadmap = self.voronoi_graph_callback()
        
        srv = Server(GlobalTuningConfig, self.parameter_callback)

    def global_planner(self, start, goal, zeta, alpha_max):
        """ Main function. Performs global plannnig

        Arguments:
            start {[type]} -- [description]
            goal {[type]} -- [description]
            zeta {double} -- [clearence threshold for pruning]
            alpha_max {[double]} -- [angle threshold for pruning]
        """        

        path = self.A_star(start.position, goal.position, self.roadmap, zeta)
        # Append start and goal
        path.insert(0, start)
        path.append(goal)
        self.plotting(self.roadmap, path, 'Path Found by A*')

        path = self.remove_collinear_waypoints(path, alpha_max, zeta)
        self.plotting(self.roadmap, path, 'Path Found After Removing Collinear Waypoints')
        
        path = self.remove_excessive_waypoints(path, zeta)
        # Calculate angles
        angles = []
        angles.append(self.start_angle)
        for i in range(1, len(path)-1):
            p1 = path[i-1]
            p2 = path[i]
            psi = math.atan2(p2.position[1] - p1.position[1], p2.position[0] - p1.position[0])
            angles.append(psi)
        angles.append(self.goal_angle)

        # Send path to local planner
        # NOTE: send by radians through z-field of pose msg. Do not use quaternions part of path msg.
        global_path = Path()
        for node, angle in zip(path, angles):
            wp = PoseStamped()
            wp.pose.position.x = node.position[0]
            wp.pose.position.y = node.position[1]
            wp.pose.position.z = angle
            global_path.poses.append(wp)

        self.pub_global_path.publish(global_path)

        # Plot in rviz
        #self.plot_nodes(path)
        self.plot_final_path(path)
        # Plot in pyplot
        self.plotting(self.roadmap, path, 'Global Path')
        self.plot_rosbag()

    def A_star(self, start_pos, goal_pos, node_list, zeta):
        """A* finds a path from start to goal. h is the heuristic function. h(n) estimates the cost to reach goal from node n.

        Arguments:
            start_pos {[type]} -- [description]
            goal_pos {[type]} -- [description]
            node_list {[type]} -- [description]
            zeta {[type]} -- [description]

        Returns:
            [list] -- [path]
        """

        # Create a start node and an goal node
        start_node = None
        goal_node = None
        min_start_dist = float("Inf")
        min_goal_dist = float("Inf")
        for node in node_list:
            d_to_start = self.euclidian_distance(start_pos, node.position)
            if d_to_start <= min_start_dist:
                min_start_dist = d_to_start
                start_node = node

            d_to_goal = self.euclidian_distance(goal_pos, node.position)
            if d_to_goal <= min_goal_dist:
                min_goal_dist = d_to_goal
                goal_node = node
            
        # Create lists for open nodes and closed nodes
        open = []
        closed = []
        # Add the start node
        open.append(start_node)

        # Loop until the open list is empty
        while len(open) > 0:
            # Sort the open list to get the node with the lowest cost first
            open.sort()
            # Get the node with the lowest cost
            current_node = open.pop(0)
            # Add the current node to the closed list
            closed.append(current_node)

            # Check if we have reached the goal, return the path
            if current_node == goal_node:
                path = []
                while current_node != start_node:
                    path.append(current_node)
                    current_node = current_node.parent
                #path.append(start) 
                # Return reversed path
                return path[::-1]

            # Loop neighbors
            for neighbour_index in current_node.neighbours:
                if node_list[neighbour_index] != current_node:

                    neighbour = node_list[neighbour_index]

                    # Check if the neighbor is in the closed list
                    if self.node_in_closed(neighbour, closed):
                        continue

                    # Check if clearence constraint is not fullfilled. Else we set the heuristic to infinity for that node. # TODO
                    if not self.check_clearence_constraints(current_node, neighbour, zeta):
                        h = float('Inf') #self.euclidian_distance(goal_node.position, neighbour.position)
                    else:
                        h = self.euclidian_distance(goal_node.position, neighbour.position) 
                    #rospy.sleep(0.5)
                    # Generate heuristics (Euclidian distance)
                    g = current_node.g + self.euclidian_distance(current_node.position, neighbour.position)
                    #neighbour.h = self.euclidian_distance(goal_node.position, neighbour.position)
                    f = g + h

                    # Check if neighbour is in open list and if it has a lower f value
                    if self.node_in_open_and_lower_f(neighbour, open):
                        continue
                    
                    neighbour.h = h
                    neighbour.g = g
                    neighbour.f = f
                    # Everything is green, add neighbour to open list
                    neighbour.parent = current_node
                    open.append(neighbour)

        # Return None, no path is found
        return None

    def remove_excessive_waypoints(self, path, zeta):
        """Removes excessive waypoints.

        Arguments:
            path {[list]} -- [the path found]
            zeta {[double]} -- [corridor width (threshold)]

        Returns:
            [list] -- [filtered path]
        """        


        i = 0
        while i < len(path) - 2:
            n1 = path[i]
            n2 = path[i+1]
            n3 = path[i+2]

            if self.check_clearence_constraints(n1, n3, zeta):
                path.remove(n2)
            else:
                i += 1

        return path

    def remove_collinear_waypoints(self, path, alpha_deg, zeta):
        """Removes collinear waypoints. Alpha is the threshold value in degrees.

        Arguments:
            path {[type]} -- [description]
            alpha_deg {[type]} -- [description]
            zeta {[type]} -- [description]

        Returns:
            [type] -- [description]
        """

        alpha_rad = alpha_deg * (math.pi/180)
        i = 0
        while i < len(path) - 2:
            n1 = path[i]
            n2 = path[i+1]
            n3 = path[i+2]
            dx_1 = math.atan2(n2.position[1] - n1.position[1], n2.position[0] - n1.position[0])
            dx_2 = math.atan2(n3.position[1] - n2.position[1], n3.position[0] - n2.position[0])
            if abs(dx_2 - dx_1) < alpha_rad:
                path.remove(n2)
            else:
                i += 1

        return path

    ### --- Utils --- ###

    def node_in_closed(self, node, closed):
        """Checks if node is in A*'s closed list.

        Arguments:
            node {[node]} -- [description]
            closed {[list]} -- [list of nodes in closed set.]

        Returns:
            [bool] -- [description]
        """        
        for n in closed:
            if node == n:
                return True
        return False
    
    def node_in_open_and_lower_f(self, node, open):
        """Checks if node is in open list of A* search.

        Arguments:
            node {[node]} -- [description]
            open {[list]} -- [list of nodes in open set]

        Returns:
            [bool] -- [description]
        """     
        for n in open:
            if (node == n and node.f > n.f):
                return True
        return False

    def node_in_nodelist(self, node, node_list):
        """Checks if node is in node list.

        Arguments:
            node {[node]} -- [description]
            node_list {[list]} -- [list of nodes]

        Returns:
            [bool] -- [description]
        """     
        for n in node_list:
            if node == n:
                return True
        return False        

    def euclidian_distance(self, p1, p2):
        """Euclidian distance between two points.

        Arguments:
            p1 {[list]} -- [first point]
            p2 {[list]} -- [second point]

        Returns:
            [type] -- [description]
        """        
        return math.sqrt( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )

    def check_clearence_constraints(self, node1, node2, zeta):
        """Checks if clearence constraints is fulfilled between two nodes.

        Arguments:
            node1 {[node]} -- [description]
            node2 {[node]} -- [description]
            zeta {[double]} -- [corridor width (threshold)]

        Returns:
            [bool] -- [if clearence i satisfied or not]
        """        
        w1_p1, w1_p2, w2_p1, w2_p2 = self.get_wall_coordinates(node1, node2, zeta)

        if self.check_wall_collision(w1_p1, w1_p2) or self.check_wall_collision(w2_p1, w2_p2):
            return False # not fullfilled. Collision
        
        return True # ok
        
    def check_wall_collision(self, p1, p2):
        """[Check collision using Bresenham's Line Algorithm.]

        Arguments:
            p1 {[list]} -- [first point]
            p2 {[list]} -- [second point]

        Returns:
            [bool] -- [if collision or not]
        """
        x1 = p1[0]
        x2 = p2[0]
        y1 = p1[1]
        y2 = p2[1]
        # Setup initial conditions
        dx = (x2 - x1)
        dy = (y2 - y1)
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        # Recalculate differentials
        dx = (x2 - x1)
        dy = (y2 - y1)
        # Calculate error
        error = int(dx / 2.0)
        ystep = self.map.info.resolution if y1 < y2 else -self.map.info.resolution
        # Iterate over bounding box generating points between start and end
        y = y1
        for x in np.arange(x1, x2, self.map.info.resolution):
            coord = (y, x) if is_steep else (x, y)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

            # CHECK COLLISION
            x_grid = coord[0]/self.map.info.resolution - self.map.info.origin.position.x/self.map.info.resolution
            y_grid = coord[1]/self.map.info.resolution - self.map.info.origin.position.y/self.map.info.resolution
            # if inside map
            if abs(int(round(x_grid))) < self.map.info.height and abs(int(round(y_grid))) < self.map.info.width:
                cell_value = self.grid[ int(round(y_grid)), int(round(x_grid)) ]            
                if cell_value == -1 or cell_value > self.occupancy_thres:
                    return True # collision

        return False # safe

    def get_wall_coordinates(self, node1, node2, zeta):
        """Get coordinates of start and end point of walls.

        Arguments:
            node1 {[node]} -- [first node]
            node2 {[node]} -- [second node]
            zeta {[type]} -- [corridor width (threshold)]

        Returns:
            [doubles] -- [4 points]
        """

        p1 = np.array(node1.position)
        p2 = np.array(node2.position)

        # unit vector
        u = (p1 - p2) / np.linalg.norm((p1 - p2))
        z = zeta * u
        # counter clockwise rotation
        rotation = np.array([-z[1], z[0]])
        
        # Wall 1
        w1_p1 = [p1[0] - rotation[0], p1[1] - rotation[1]]
        w1_p2 = [p2[0] - rotation[0], p2[1] - rotation[1]]
        # Wall 2
        w2_p1 = [p1[0] + rotation[0], p1[1] + rotation[1]]
        w2_p2 = [p2[0] + rotation[0], p2[1] + rotation[1]]

        return w1_p1, w1_p2, w2_p1, w2_p2

    def quaternion_to_euler(self, x, y, z, w):
        """Quaternions to euler.

        Arguments:
            x {[double]} -- [quaternion 1]
            y {[double]} -- [quaternion 2]
            z {[double]} -- [quaternion 3]
            w {[double]} -- [quaternion 4]

        Returns:
            [type] -- [euler angles]
        """  
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z

    @staticmethod
    def ssa(angle):
        """Smallest signed angle. Maps angle into interval [-pi pi]

        Arguments:
            angle {[double]} -- [angle]

        Returns:
            [double] -- [wrapped angle]
        """        
        wrpd_angle = (angle + math.pi) % (2*math.pi) - math.pi
        return wrpd_angle

    ### --- Plotting in Rviz and pyplot --- ###

    def plot_nodes(self, node_list):
        """Plot nodes in Rviz

        Arguments:
            node_list {[list]} -- [list of nodes]
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
        points.scale.x = 1 
        points.scale.y = 1
        points.color.r = 0.0
        points.color.g = 1.0
        points.color.b = 0.0
        points.color.a = 1.0
        points.lifetime = rospy.Duration()

        for node in node_list:
            p1 = Point()
            p1.x = node.position[0]
            p1.y = node.position[1]
            p1.z = 0.04
            points.points.append(p1)
        
        self.pub_plot_nodes.publish(points)

    def plot_final_path(self, node_list):
        path = Marker()

        # edge between nodes
        path = Marker()
        path.header.frame_id = "map"
        path.header.stamp = rospy.get_rostime()
        path.ns = "markers"
        path.id = 1
        path.type = path.LINE_STRIP
        path.action = path.ADD
        path.scale.x = 0.1
        path.color.a = 1.0
        
        path.color.r = 1.0
        path.color.g = 0.0
        path.color.b = 0.0

        path.lifetime = rospy.Duration()
        path.pose.orientation.w = 1.0

        for node in node_list:
            p1 = Point()
            p1.x = node.position[0]
            p1.y = node.position[1]
            p1.z = 0.03
            path.points.append(p1)

        self.pub_plot_path.publish(path)

    def plotting(self, roadmap, path, title):
        """ Plotting of global path found in pyplot.

        Arguments:
            roadmap {[list]} -- [the voronoi roadmap]
            path {[list]} -- [the path found]
            title {[string]} -- [title of plot]
        """
        plt.rc('font', family='serif')
        plt.rc('text', usetex=True)
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

        # Plot Voroni diagram
        i = 1
        for node in roadmap:
            #ax.plot(node.position[0], node.position[1], marker = ".", markersize=3)
            for neighbour_idx in node.neighbours:
                neighbour = roadmap[neighbour_idx]

                path_x = [node.position[0], neighbour.position[0]] 
                path_y = [node.position[1], neighbour.position[1]]
                if i == 1: # plot with label once
                    ax.plot(path_x, path_y, 'r-', linewidth=1, label=r'Voronoi roadmap')
                    i += 1
                else:
                    ax.plot(path_x, path_y, 'r-', linewidth=1)
        
        # Plot global path
        i = 1
        global_path_x = []
        global_path_y = []
        for node in path:
            global_path_x.append(node.position[0])
            global_path_y.append(node.position[1])

        ax.plot(global_path_x, global_path_y, 'g.-', linewidth=1, label=r'Global path')
        
        ax.plot(global_path_x[0], global_path_y[0], 'b.', markersize=8, label=r'Start/goal')
        ax.plot(global_path_x[-1], global_path_y[-1], 'b.', markersize=8)

        plt.xlabel(r'East $[m]$', fontsize = 12)
        plt.ylabel(r'North $[m]$', fontsize = 12)
        plt.title(title, fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right')
        plt.tight_layout()

        plt.show()

    def plot_rosbag(self):
        """pyplot of rosbag.
        """        
        plt.rc('font', family='serif')
        plt.rc('text', usetex=True)
        # Plot pgm map
        fig, ax = plt.subplots()
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        x_scale =  resolution * width
        y_scale = resolution * height
        org_x = self.map.info.origin.position.x
        org_y = self.map.info.origin.position.y
        imgplot = ax.imshow(np.flipud(self.grid), extent=[org_x, x_scale + org_x, org_y, y_scale + org_y])
        imgplot.set_cmap('Greys')        
        
        #plt.axis('off')
        #plt.savefig("/home/revolt/fulleps.eps", bbox_inches='tight')
        #plt.tight_layout()
        #plt.show()

        path = '/home/revolt/'
        bag = rosbag.Bag(path + 'full1.bag')
        rad2deg = 180/math.pi
        X = [ msg.linear.x for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
        Y = [ msg.linear.y for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
        psi = [ msg.angular.z for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
        T = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
        x = X
        y = Y

        # x_org = X[0] #152.39578482
        # y_org = Y[0] #1128.15407673
        # Set origin to zero
        # for x_pos, y_pos in zip(X, Y):
        #     x.append(x_pos - x_org)
        #     y.append(y_pos - y_org)

        X_d = [ msg.eta_d.x for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        Y_d = [ msg.eta_d.y for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        psi_d = [ rad2deg * msg.eta_d.theta for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        T_d = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        x_d = X_d
        y_d = Y_d

        # Set origin to zero
        # for x_pos, y_pos in zip(X_d, Y_d):
        #     x_d.append(x_pos - x_org)
        #     y_d.append(y_pos - y_org)
        # NED
        #plt.figure(1)
        #plt.grid(True)
        #plt.axis('equal')
        plt.plot(x_d[:len(x_d)-10], y_d[:len(y_d)-10], 'g--', LineWidth=1.5, label=r'$\mathbf{p}_d^n$')
        plt.plot(x, y, Color = 'b', LineWidth=1.5, label=r'$\mathbf{p}^n$', alpha=.7)

        nth = 150
        i = 0
        for x_i, y_i, psi_i in zip(x, y, psi):        
            if i % nth == 0:
                m = MarkerStyle("^")
                m._transform.scale(0.6, 1)
                m._transform.rotate_deg(psi_i-90)  
                if i == 0:
                    plt.scatter(x_i, y_i, s=225, marker = m, color = 'orange', label=r'Vessel')
                else:
                    plt.scatter(x_i, y_i, s=225, marker = m, color = 'orange')
            i += 1

        plt.xlabel(r'East $[m]$', fontsize = 12)
        plt.ylabel(r'North $[m]$', fontsize = 12)
        plt.title(r'Position in the Horizontal Plane', fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right')
        
        plt.tight_layout()

        # Pose vs t
        plt.figure(2)
        plt.subplot(311)
        plt.grid(True)
        plt.plot( T_d[:len(T_d)-10] , y_d[:len(y_d)-10],  'g--', LineWidth = 1.5, label=r'$\mathbf{\eta}_d(t)$')
        plt.plot( T , y, Color = 'b', LineWidth = 1.5, label=r'$\mathbf{\eta}(t)$', alpha=.7)
        plt.ylabel(r'$x(t) \: [m]$', fontsize = 12)
        plt.title(r'North Position', fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right')

        plt.subplot(312)
        plt.grid(True)
        plt.plot( T_d[:len(T_d)-10] , x_d[:len(x_d)-10],  'g--', LineWidth = 1.5)
        plt.plot( T , x, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$y(t) \: [m]$', fontsize = 12)
        plt.title(r'East Position', fontsize = 12)

        plt.subplot(313)
        plt.grid(True)
        plt.plot( T_d[:len(T_d)-10] , psi_d[:len(y_d)-10],  'g--', LineWidth = 1.5)
        plt.plot( T , psi, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$\psi(t) \: [deg]$', fontsize = 12)
        plt.title(r'Attitude', fontsize = 12)

        plt.xlabel(r'$t \: [s]$', fontsize = 12)    
        plt.tight_layout()


        vel_X = [ msg.linear.x for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
        vel_Y = [ msg.linear.y for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
        vel_psi = [ msg.angular.z for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
        vel_T = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]

        vel_X_d = [ msg.dot_eta_d.x for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        vel_Y_d = [ msg.dot_eta_d.y for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        vel_psi_d = [ msg.dot_eta_d.theta for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        vel_T_d = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
        v_ref = [ msg.v_ref for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]

        # Velocities vs t
        plt.figure(3)
        plt.subplot(311)
        plt.grid(True)
        plt.plot( vel_T , vel_X, Color = 'b', LineWidth = 1.5, label=r'$\mathbf{\nu}(t)$', alpha=.7)
        plt.ylabel(r'$u(t) \: [m/s]$', fontsize = 12)
        plt.title(r'Surge Speed', fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right')

        plt.subplot(312)
        plt.grid(True)
        plt.plot( vel_T , vel_Y, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$v(t) \: [m/s]$', fontsize = 12)
        plt.title(r'Sway Speed', fontsize = 12)

        plt.subplot(313)
        plt.grid(True)
        plt.plot( vel_T , vel_psi, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$r(t) \: [deg/s]$', fontsize = 12)
        plt.title(r'Yaw Rate', fontsize = 12)

        plt.xlabel(r'$t \: [s]$', fontsize = 12)
        plt.tight_layout()

        ### Forces

        tau_x = [ msg.force.x for (topic, msg, t) in bag.read_messages(topics=['/tau_controller'])]
        tau_y = [ msg.force.y for (topic, msg, t) in bag.read_messages(topics=['/tau_controller'])]
        tau_n = [ msg.torque.z for (topic, msg, t) in bag.read_messages(topics=['/tau_controller'])]
        tau_t = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/tau_controller'])]

        tau_x_c = [ msg.force.x for (topic, msg, t) in bag.read_messages(topics=['/tau_commanded'])]
        tau_y_c = [ msg.force.y for (topic, msg, t) in bag.read_messages(topics=['/tau_commanded'])]
        tau_n_c = [ msg.torque.z for (topic, msg, t) in bag.read_messages(topics=['/tau_commanded'])]
        tau_t_c = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/tau_commanded'])]

        plt.figure(4)
        plt.subplot(311)
        plt.grid(True)
        plt.plot( tau_t , tau_x, 'g--', LineWidth = 1.5, label=r'$\mathbf{\tau}_d(t)$')
        plt.plot( tau_t_c , tau_x_c, 'b', LineWidth = 1.5, label=r'$\mathbf{\tau}(t)$', alpha=.7)
        plt.ylabel(r'$X(t) \: [N]$', fontsize = 12)
        plt.title(r'Surge Force', fontsize = 12)
        plt.legend(shadow = True, loc = 'upper right')

        plt.subplot(312)
        plt.grid(True)
        plt.plot( tau_t , tau_y, 'g--', LineWidth = 1.5)
        plt.plot( tau_t_c , tau_y_c, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$Y(t) \: [N]$', fontsize = 12)
        plt.title(r'Sway Force', fontsize = 12)

        plt.subplot(313)
        plt.grid(True)
        plt.plot( tau_t , tau_n, 'g--', LineWidth = 1.5)
        plt.plot( tau_t_c , tau_n_c, Color = 'b', LineWidth = 1.5, alpha=.7)
        plt.ylabel(r'$N(t) \: [Nm]$', fontsize = 12)
        plt.title(r'Yaw Moment', fontsize = 12)

        plt.xlabel(r'$t \: [s]$', fontsize = 12)    
        plt.tight_layout()

        #plt.draw()
        plt.pause(400)
        plt.show()

    ### ROS callbacks ###

    def parameter_callback(self, config, level):
        """ Callback function for updating parameters.

        Parameters
        ----------
        config : ParameterGenerator()
            configuration parameters

        """
        start_pos = ast.literal_eval(config.start)
        goal_pos = ast.literal_eval(config.goal)

        self.start_node = self.Node(-1, start_pos, [])
        self.goal_node = self.Node(-1, goal_pos, [])
        self.zeta = config.clearence_threshold
        self.alpha_max = config.angle_threshold
        self.start_angle = math.radians(config.start_angle)
        self.goal_angle = math.radians(config.goal_angle)
        self.occupancy_thres = config.occupancy_thres

        if config.execute_plan == True:
        ### Run global planner ###
            self.global_planner(self.start_node, self.goal_node, self.zeta, self.alpha_max)

        return config

    def voronoi_graph_callback(self):
        """Revieves the Vornoi graph msg. Creates nodes in terms of waypoints to be searched by A*.

        Returns:
            [list] -- [roadmap]
        """        
        graph_msg = rospy.wait_for_message("segments", Graph)

        resolution = graph_msg.resolution
        x_org = graph_msg.origin.position.x
        y_org = graph_msg.origin.position.y

        node_list = [] # roadmap

        for node_msg in graph_msg.vertices:
            id = node_msg.id
            # front
            x_f = node_msg.path[0].x * resolution + x_org
            y_f = node_msg.path[0].y * resolution + y_org
            # back
            # x_b = node_msg.path[-1].x * resolution + x_org
            # y_b = node_msg.path[-1].y * resolution + y_org
            
            neighbours = []
            neighbours.extend(node_msg.successors)
            neighbours.extend(node_msg.predecessors)
            
            node = self.Node(id, [x_f, y_f], neighbours)
            node_list.append(node)

        # filter
        for node1 in node_list:
            for node2 in node_list:
                if node1 == node2:
                    node1.neighbours.extend(node2.neighbours)
                    node1.neighbours = list(set(node1.neighbours))

                    node2.neighbours.extend(node1.neighbours)
                    node2.neighbours = list(set(node2.neighbours))

        id = len(node_list)
        for node_msg in graph_msg.vertices:
            # front
            # x_f = node_msg.path[0].x * resolution + x_org
            # y_f = node_msg.path[0].y * resolution + y_org
            # back
            x_b = node_msg.path[-1].x * resolution + x_org
            y_b = node_msg.path[-1].y * resolution + y_org
            
            # At least one neighbour, which is the node corresponding to the front.
            neighbours = [node_msg.id]

            node = self.Node(id, [x_b, y_b], neighbours)

            # Search thorugh and see if node exists in already. If: drop the node.
            if self.node_in_nodelist(node, node_list):
                continue
            # Else:
            node_list[node_msg.id].neighbours.append(id)
            node_list.append(node)
            id += 1

        return node_list

    def callback_map(self, occupancy_grid_msg):
        """Callback for map listener.

        Arguments:
            occupancy_grid_msg {[ros msg]} -- [nav_msg OccupancyGrid]
        """        
        self.map = occupancy_grid_msg

    def callback_rviz_start(self, start_msg):
        """Start pose for global planner from rviz. TODO: Chage to posestamped

        Arguments:
            start_msg {PoseWithCovarianceStamped} -- [ros msg]
        """        
        # Convert quaternions to Euler angles.
        _, _, yaw = self.quaternion_to_euler(start_msg.pose.pose.orientation.x, start_msg.pose.pose.orientation.y, start_msg.pose.pose.orientation.z, start_msg.pose.pose.orientation.w)

        self.start_node = self.Node(-1, [start_msg.pose.pose.position.x, start_msg.pose.pose.position.y], [])
        self.start_angle = yaw

    def callback_rviz_goal(self, goal_msg):
        """Goal pose for global planner from rviz

        Arguments:
            start_msg {PoseStamped} -- [ros msg]
        """      
        # Convert quaternions to Euler angles.
        _, _, yaw = self.quaternion_to_euler(goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w)

        self.goal_node = self.Node(-1, [goal_msg.pose.position.x, goal_msg.pose.position.y], [])
        self.goal_angle = yaw

        # Execute plan
        self.global_planner(self.start_node, self.goal_node, self.zeta, self.alpha_max)

if __name__ == '__main__':

    planner = GlobalPlanner()

    rospy.spin()

    