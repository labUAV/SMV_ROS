#!/usr/bin/env python

"""
This script conveniently plots the robot trajectory in a X-Y graph

@author Davide Carminati

Copyright (C) 2024 Davide Carminati

This program is free software: you can redistribute it and/or modify 
it under the terms of the GNU General Public License as published by 
the Free Software Foundation, either version 3 of the License, or 
(at your option) any later version. 

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
GNU General Public License for more details. 

You should have received a copy of the GNU General Public License 
along with this program. If not, see <https://www.gnu.org/licenses/>. 
"""

import rospy
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt

import numpy as np
import collections


fig = plt.figure()

x = collections.deque(maxlen=200000)
y = collections.deque(maxlen=200000)
vx = collections.deque(maxlen=200000)
vy = collections.deque(maxlen=200000)

def callback(odom:Odometry):

    global x, y

    if rospy.has_param('/goal_position_x') and rospy.has_param('/goal_position_y'):
        x_goal = rospy.get_param('/goal_position_x')
        y_goal = rospy.get_param('/goal_position_y')
        goal_tol = rospy.get_param('/goal_tolerance')
        angles = np.linspace(-2 * np.pi, 2 * np.pi, 100)
        angles = np.array(angles[:, np.newaxis])
        goal_circle = np.hstack( (x_goal + goal_tol * np.cos(angles), y_goal + goal_tol * np.sin(angles)) )

    ######################################################
    ## Add a piece of code to plot the obstacles as well #
    ######################################################

    # if rospy.has_param('/number_obstacles') and rospy.has_param('obstacle_position'):
    #     if rospy.get_param('/number_obstacles') == 0:
    #         rospy.logwarn('Check obstacles properties')
    #     # Plot obstacles...

    x.append(odom.pose.pose.position.x)
    y.append(odom.pose.pose.position.y)
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y

    plt.clf()
    ax1 = fig.subplots()
    plt.plot(x, y, 'b--')
    plt.quiver(x[-1], y[-1], vx, vy)
    plt.plot(goal_circle[:, 0], goal_circle[:, 1], 'g')
    plt.rc('xtick', labelsize=12)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=12)

    # If you want to use LaTex to generate the text in the plot 
    # uncomment the following line
    # plt.rcParams['text.usetex'] = True

    plt.grid(True)
    ax1.set_xlabel("$x [m]$", fontsize=12)
    ax1.set_ylabel("$y [m]$", fontsize=12)
    ax1.set_ylim(-1, x_goal + 0.5)
    ax1.set_xlim(-1, y_goal + 0.5)
    plt.title("XY plot")

    plt.draw()
    plt.pause(0.01)

def listener():

    rospy.init_node('plot_utility', anonymous=True)
    
    rospy.Subscriber("/estimated_state", Odometry, callback, queue_size=1)

    plt.show()
    rospy.spin()

if __name__ == '__main__':

    listener()