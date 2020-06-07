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
    plt.rc('text', usetex=True)
    bag = rosbag.Bag('/home/revolt/bags/s2.bag')
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
    plt.figure(1)
    plt.grid(True)    
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

    plt.axis('equal')
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
    plt.plot( T[:len(T)-10] , y[:len(y)-10], Color = 'b', LineWidth = 1.5, label=r'$\mathbf{\eta}(t)$', alpha=.7)
    plt.ylabel(r'$x(t) \: [m]$', fontsize = 12)
    plt.title(r'North Position', fontsize = 12)
    plt.legend(shadow = True, fontsize = 12)

    plt.subplot(312)
    plt.grid(True)
    plt.plot( T_d[:len(T_d)-10] , x_d[:len(x_d)-10],  'g--', LineWidth = 1.5)
    plt.plot( T[:len(T)-10] , x[:len(x)-10], Color = 'b', LineWidth = 1.5, alpha=.7)
    plt.ylabel(r'$y(t) \: [m]$', fontsize = 12)
    plt.title(r'East Position', fontsize = 12)

    plt.subplot(313)
    plt.grid(True)
    plt.plot( T_d[:len(T_d)-10] , psi_d[:len(y_d)-10],  'g--', LineWidth = 1.5)
    plt.plot( T[:len(T)-10] , psi[:len(psi)-10], Color = 'b', LineWidth = 1.5, alpha=.7)
    plt.ylabel(r'$\psi(t) \: [deg]$', fontsize = 12)
    plt.title(r'Attitude', fontsize = 12)

    plt.xlabel(r'$t \: [s]$', fontsize = 12)    
    plt.tight_layout()


    vel_X = [ msg.linear.x for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_Y = [ msg.linear.y for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_psi = [ rad2deg * msg.angular.z for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]
    vel_T = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/observer/nu/body'])]

    vel_X_d = [ msg.dot_eta_d.x for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
    vel_Y_d = [ msg.dot_eta_d.y for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
    vel_psi_d = [ rad2deg * msg.dot_eta_d.theta for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
    vel_T_d = [ t.to_time() for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]
    v_ref = [ msg.v_ref for (topic, msg, t) in bag.read_messages(topics=['/GuidanceSystem/HybridPathSignal'])]

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
    plt.legend(shadow = True, fontsize = 12)

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
    #plt.pause(400)
    plt.show()