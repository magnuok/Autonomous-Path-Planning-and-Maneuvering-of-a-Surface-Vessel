#!/usr/bin/env python3
"""
Path planning Sample Code with RT-RRT*

uthor: Magnus Knædal
Date: 10.06.2020

"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import distance

def static_var(**kwargs):
    """
    This function creates decorator. Used for counting recursive calls.
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate

class prettyfloat(float):
    """
    Class for printing float with given precision
    """
    def __repr__(self):
        return "%0.2f" % self

class RRT_utils:
    """
    Class for util. function for  RT-RRTstar
    """
    
    def check_obstacle_collision(self, node1, node2, obstacle_list):
        """
        Checks if the segments between two nodes collides with any obstacles.
        """
        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y

        px = x2-x1
        py = y2-y1

        norm = px*px + py*py

        for obstacle in obstacle_list:
            ox = obstacle[0]
            oy = obstacle[1]
            r = obstacle[2]
            if norm == 0: # Points are equal
                dist = (x1-ox)**2 + (y1-oy)**2
            else:
                u =  ((ox - x1) * px + (oy - y1) * py) / float(norm)

                if u > 1:
                    u = 1
                elif u < 0:
                    u = 0

                x = x1 + u * px
                y = y1 + u * py

                dx = x - ox
                dy = y - oy

                dist = dx*dx + dy*dy

            if dist <= r**2:
                return False  # collision
        
        return True # safe

    def get_grid_cells(self, nodes):
        res = self.map.info.resolution
        grid_cells = []
        
        for node in nodes:
            x = node[0]/res - self.map.info.origin.position.x/res #  - 1
            y = node[1]/res - self.map.info.origin.position.y/res #  - 1
            grid_cells.append(self.grid[ int(round(y)), int(round(x)) ])
        return grid_cells

    def get_grid_cell(self, x, y):
        """
        Returns the value of a grid cell if inside dimensions, else it returns None.
        """

        i = x/self.map.info.resolution - self.map.info.origin.position.x/self.map.info.resolution
        j = y/self.map.info.resolution - self.map.info.origin.position.y/self.map.info.resolution        
        #res = self.map.info.resolution
        #i = x/res + self.map.info.origin.position.x/res - 1
        #j = y/res + self.map.info.origin.position.y/res - 1
        #print("i = %f, j = %f" % (i,j))
        
        if int(round(i)) < self.map.info.height and int(round(j)) < self.map.info.width: 
            cell = self.grid[ int(round(j)), int(round(i)) ]
        else:
            cell = None

        return cell

    def get_free_space(self, map, grid):
        resolution = map.info.resolution
        width = map.info.width
        height = map.info.height

        # add the index of element numbers that are free
        for i in range(0, width):
            for j in range(0, height):
                if grid[i,j] != 100 and grid[i,j] != -1:
                    self.index_free_space_list.append((i,j))

    def get_ned_position(self, coordinates):
        """
        Grid cell to ned position. index = (x,y)
        """
        resolution = self.map.info.resolution

        # Calculate distance
        x = coordinates[0] * resolution + self.map.info.origin.position.x
        y = coordinates[1] * resolution + self.map.info.origin.position.y

        return [x, y]

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_wall_collision(self, node1, node2):
        """
        Check collision using Bresenham directly.
        """
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
        """
        x1 = node1.x
        y1 = node1.y
        x2 = node2.x
        y2 = node2.y

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
            #x_grid = coord[0]/self.map.info.resolution + self.map.info.origin.position.x/self.map.info.resolution - 1 
            #y_grid = coord[1]/self.map.info.resolution + self.map.info.origin.position.y/self.map.info.resolution - 1
            x_grid = coord[0]/self.map.info.resolution - self.map.info.origin.position.x/self.map.info.resolution
            y_grid = coord[1]/self.map.info.resolution - self.map.info.origin.position.y/self.map.info.resolution

            if abs(int(round(x_grid))) < self.map.info.height and abs(int(round(y_grid))) < self.map.info.width:
                cell_value = self.grid[ int(round(y_grid)), int(round(x_grid)) ]
                if cell_value == -1 or cell_value >= self.occupied_thres:
                    return False # collision
    
        return True # safe

    @staticmethod
    def euclidian_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        alpha = math.atan2(dy, dx)
        return d, alpha

    @staticmethod
    def euclidian_distance(from_node, to_node):
        return math.sqrt( (to_node.x - from_node.x)**2 + (to_node.y - from_node.y)**2 )

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    @staticmethod
    def rotation_to_world_frame(a1):
        """
         Given two poses as the focal points of a hyperellipsoid, xfrom, xto ∈ X, the function RotationToWorldFrame (xfrom, xto) 
         returns the rotation matrix, R element in SO(2), from the hyperellipsoid-aligned frame to the NED-frame
        """
        # first column of idenity matrix transposed
        id1_t = np.array([1.0, 0.0, 0.0]).reshape(1, 3)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, True, True)
        R = np.dot(np.dot(U, np.diag(
            [1.0, 1.0, np.linalg.det(U) * np.linalg.det(np.transpose(Vh))])), Vh)
        
        return R

    @staticmethod
    def compute_sampling_space(start_node, goal_node):
        """
        Computes values for the heuristic sampling domain, formed by an ellipse.
        Sample space is defined by cBest
        cMin is the minimum distance between the start point and the goal
        xCenter is the midpoint between the start and the goal
        cBest changes when a new path is found
        """
        cMin = math.sqrt(pow(start_node.x - goal_node.x, 2)
                         + pow(start_node.y - goal_node.y, 2))
        xCenter = np.array([[(start_node.x + goal_node.x) / 2.0],
                            [(start_node.y + goal_node.y) / 2.0], [0]])
        a1 = np.array([[(goal_node.x - start_node.x) / cMin],
                       [(goal_node.y - start_node.y) / cMin], [0]])

        etheta = math.atan2(a1[1], a1[0])

        return cMin, xCenter, a1, etheta

    @staticmethod
    def sample_unit_ball():
        """
        The function, sample_unit_ball returns a uniform sample from the volume of an circle of 
        unit radius centred at the origin.
        """
        a = random.random()
        b = random.random()

        if b < a:
            a, b = b, a

        sample = (b * math.cos(2 * math.pi * a / b),
                  b * math.sin(2 * math.pi * a / b))

        return np.array([[sample[0]], [sample[1]], [0]])

    def get_path_len(self, path):
        pathLen = 0
        for i in range(1, len(path)):
            node1 = path[i]
            node2 = path[i - 1]
            pathLen += self.euclidian_distance(node1, node2)

        return pathLen

    @staticmethod
    def ssa(angle):
        """
        Smallest signed angle. Maps angle into interval [-pi pi]
        """
        wrpd_angle = (angle + math.pi) % (2*math.pi) - math.pi
        return wrpd_angle

    @static_var(counter=0)
    def get_sum_c_c(self, from_node):
        """
        Finds sum of curvature cost, recursively. The static variable keeps track of depth/#parents
        """
        # stash counter in the function itself
        RRT_utils.get_sum_c_c.counter += 1
        if from_node.parent == None:
            return 0
        return from_node.cost + self.get_sum_c_c(from_node.parent)

    @staticmethod
    def get_max_kappa(node):
        """
        Finds maximum curvature from node to root, recursively.
        """
        if node.parent == None:
            return 0
        return max(node.cost, RRT_utils.get_max_kappa(node.parent))

    @staticmethod
    def get_min_obstacle_distance(node, obstacleList):
        """
        Finds minimum distance to obstacle from node.
        """
        d_list = [math.sqrt((obstacle[0] - node.x)**2 + (obstacle[1] - node.y)**2 ) for obstacle in obstacleList]
        return min(d_list)

    def calc_heuristic(self, node, goal_node):
         return self.euclidian_distance(node, goal_node)

    def get_heuristic(self, node, goal_node, visisted_set):
        """
        Returns heuristic cost for node.
        """
        if node in visisted_set:
            return float('Inf')
        else:
            return self.euclidian_distance(node, goal_node)

    @staticmethod
    def paths_are_equal(list1, list2):
        return all(elem1 == elem2 for elem1, elem2 in zip(list1, list2))

    def unblock_parents(self, node, visisted_set):

        while node.parent in visisted_set:
            visisted_set.remove(node.parent)
            node = node.parent

    def check_if_all_children_blocked(self, node, visited_set):
        """
        Check if visited_set contains all elements in node.children and if all children
        have cost == inf.
        """
        all_inf = all([child.cost == float('Inf')  for child in node.children])
        all_in_visited = all(elem in visited_set for elem in node.children)
        return (all_inf or all_in_visited)

    def generate_K_step_path(self, target, root):
        """
        Generates path from root+1 to target.
        """
        path = []
        node = target
        while node.parent is not None:
            path.append(node)
            node = node.parent
        
        path.reverse()
        return path

    def generate_final_path(self, target):
        """
        Generates path from root+1 up to target and adds goal_node aswell.
        """
        path = [self.goal_node]

        node = target
        while node.parent is not None:
            path.append(node)
            node = node.parent

        path.reverse()

        return path

    def get_minimum_cost_child(self, node, goal_node, visisted_set):
        """
        Gets child of node with minimum total cost f = g + h. 
        """
        min_cost_child = None
        min_cost = float('Inf')
        for child in node.children:
            cost = child.cost + self.get_heuristic(child, goal_node, visisted_set)
            if cost <= min_cost:
                min_cost = cost
                min_cost_child = child
        
        return min_cost_child


    #### Unused ###

    def print_path(self, path, name):
        print(name)
        print("  x,   y,  alpha,  kappa,  d,  cost")
        for node in path:
            i = [node.x, node.y, math.degrees(node.alpha), node.kappa, node.d, node.cost]
            i = map(prettyfloat, i)
            print(list(i))


    def print_children(self, path):
        print("Children:")
        for node in path:
                string = ""
                for child in node.children:
                    i = [child.x, child.y]
                    i = map(prettyfloat, i)
                    string += str(list(i)) + ", "
                print(string)        
