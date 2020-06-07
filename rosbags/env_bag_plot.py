#!/usr/bin/env python3
"""
Script for plotting rosbag data.

Author: Magnus Knaedal
Date: 10.06.2020

"""
import rosbag
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
import math
import numpy as np
import matplotlib.image as mpimg



if __name__ == '__main__':

    plt.rc('font', family='serif')
    bag = rosbag.Bag('/home/revolt/environment.bag')
    rad2deg = 180/math.pi

    X = [ msg.linear.x for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
    Y = [ msg.linear.y for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
    psi = [ msg.angular.z for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
    T = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/observer/eta/ned'])]
    x = []
    y = []

    x_org = 152.39578482
    y_org = 1128.15407673

    # Set origin to zero
    for x_pos, y_pos in zip(X, Y):
        x.append(x_pos + x_org)
        y.append(y_pos + y_org)

    # NED
    plt.figure(1)
    plt.grid(True)
    plt.plot(x, y, Color = 'b', LineWidth=1.5, label=r'$\mathbf{p}^n$', alpha=.7)

    nth = 300
    i = 0
    for x_i, y_i, psi_i in zip(x, y, psi):        
        if i % nth == 0:
            m = MarkerStyle("^")
            m._transform.scale(0.8, 2)
            m._transform.rotate_deg(psi_i-90)  
            if i == 0:
                plt.scatter(x_i, y_i, s=225, marker = m, color = 'orange', label=r'Vessel')
            else:
                plt.scatter(x_i, y_i, s=225, marker = m, color = 'orange')
        i += 1

    plt.axis('equal')
    plt.xlabel(r'East $[m]$', fontsize = 12)
    plt.ylabel(r'North $[m]$', fontsize = 12)
    plt.title(r'Position in the Horizontal Plane', fontsize = 12)
    
    plt.legend(shadow = True, loc = 'center right')

    plt.tight_layout()

    # Pose vs t
    plt.figure(2)
    plt.subplot(311)
    plt.grid(True)
    plt.plot( T[:len(T)-10] , y[:len(y)-10], Color = 'b', LineWidth = 1.5, label=r'$\mathbf{\eta}(t)$', alpha=.7)
    plt.ylabel(r'$x(t) \: [m]$', fontsize = 12)
    plt.title(r'North Position', fontsize = 12)
    plt.legend(shadow = True, fontsize = 12)

    plt.subplot(312)
    plt.grid(True)
    plt.plot( T[:len(T)-10] , x[:len(x)-10], Color = 'b', LineWidth = 1.5, alpha=.7)
    plt.ylabel(r'$y(t) \: [m]$', fontsize = 12)
    plt.title(r'East Position', fontsize = 12)

    plt.subplot(313)
    plt.grid(True)
    plt.plot( T[:len(T)-10] , psi[:len(psi)-10], Color = 'b', LineWidth = 1.5, alpha=.7)
    plt.ylabel(r'$\psi(t) \: [deg]$', fontsize = 12)
    plt.title(r'Attitude', fontsize = 12)

    plt.xlabel(r'$t \: [s]$', fontsize = 12)    
    plt.tight_layout()


    vel_X = [ msg.linear.x for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_Y = [ msg.linear.y for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_psi = [ rad2deg * msg.angular.z for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_T = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]

    # Velocities vs t
    plt.figure(3)
    plt.subplot(311)
    plt.grid(True)
    plt.plot( vel_T , vel_X, Color = 'b', LineWidth = 1.5, label=r'$\mathbf{\nu}(t)$', alpha=.7)
    plt.ylabel(r'$u(t) \: [m/s]$', fontsize = 12)
    plt.title(r'Surge Speed', fontsize = 12)
    plt.legend(shadow = True, fontsize = 12)

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


    plt.show()