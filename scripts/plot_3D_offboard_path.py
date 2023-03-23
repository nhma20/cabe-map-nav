#!/usr/bin/env python3
###############################################################################
# Includes
###############################################################################

import struct
from xml.sax.handler import property_xml_string
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleOdometry
from iii_interfaces.msg import PowerlineDirection
from std_msgs.msg import Int32

import cv2 as cv
from cv_bridge import CvBridge

import numpy as np

import time
import copy
import sys
import os
import math 
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from mpl_toolkits.mplot3d import Axes3D

from threading import Thread, Lock

from sshkeyboard import listen_keyboard



###############################################################################
#  Defines
###############################################################################
global image_width
image_width  = 640 #1920
global image_height
image_height  = 480 #1080
global img_dims
img_dims = (image_width, image_height)



###############################################################################
# Class
###############################################################################

class IdDrawer(Node):

    key_thread = None

    def __init__(self):
        self.xyz_error_list_ = []

        self.colors = ['w','y','limegreen','b','r','m']
        self.legend_str = ['None','Take-off to\n cable 1','Go to\n cable 2','Go to\n cable 1','Go to\n cable 2','Go to\n cable 3']

        self.ego_lock_ = Lock()
        self.pl_lock_ = Lock()
        self.dir_lock = Lock()
        self.all_lock_ = Lock()

        self.cur_id_ = -2
        self.new_id_cnt_ = 0

        self.key_thread = Thread(target=self.keylistener, args=())

        self.ego_xyz_ = []
        self.pl_xyz_ = []
        self.pl_dir_ = 0.22

        self.all_x_error_ = [[],[],[],[],[],[],[],[],[],[]]
        self.all_y_error_ = [[],[],[],[],[],[],[],[],[],[]]
        self.all_z_error_ = [[],[],[],[],[],[],[],[],[],[]]

        super().__init__("image_drawer")

        self.pl_sub_ = self.create_subscription(
            Int32,
            "/typed_ID",	
            self.on_id_msg,
            10
        )

        self.pl_sub_ = self.create_subscription(
            PointStamped,
            "/ID_point",	
            self.on_pl_msg,
            10
        )

        self.odo_sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/vehicle_odometry/out",	
            self.on_odo_msg,
            10
        )


        self.pl_dir_sub = self.create_subscription(
            PowerlineDirection,
            "/hough_cable_yaw_angle",	
            self.on_pl_dir_msg,
            10
        )


        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.store_xyz)


    def on_id_msg(self, msg):
        if self.cur_id_ != msg.data:
            self.new_id_cnt_ = self.new_id_cnt_ + 1
            self.cur_id_ = msg.data


    def on_pl_msg(self, msg):
        if self.pl_lock_.acquire(blocking=True):
            x_tmp0 = msg.point.x
            y_tmp0 = msg.point.y
            z_tmp0 = msg.point.z
            self.pl_xyz_ = [x_tmp0, y_tmp0, z_tmp0]
            self.pl_lock_.release()


    def on_odo_msg(self, msg):
        if self.ego_lock_.acquire(blocking=True):
            x_tmp1 = msg.x
            y_tmp1 = msg.y
            z_tmp1 = msg.z
            self.ego_xyz_ = [x_tmp1, y_tmp1, z_tmp1]
            self.ego_lock_.release()


    def on_pl_dir_msg(self, msg):
        if self.dir_lock.acquire(blocking=True):
            self.pl_dir_ = -msg.angle
            self.dir_lock.release()


    def store_xyz(self):
        if self.pl_lock_.acquire(blocking=True):
            pl_latest_xyz = self.pl_xyz_
            self.pl_lock_.release()
        
        if self.ego_lock_.acquire(blocking=True):
            ego_latest_xyz = self.ego_xyz_
            self.ego_lock_.release()

        if self.all_lock_.acquire(blocking=True):
            if pl_latest_xyz != [] and ego_latest_xyz != []:
                self.all_x_error_[self.new_id_cnt_].append(ego_latest_xyz[0])
                self.all_y_error_[self.new_id_cnt_].append(-ego_latest_xyz[1])
                self.all_z_error_[self.new_id_cnt_].append(-ego_latest_xyz[2])
            self.all_lock_.release()


    def draw_image(self):
    
        if self.all_lock_.acquire(blocking=True):
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')


            for i in range(1, len(self.colors)):
                ax.plot(self.all_x_error_[i], self.all_y_error_[i], self.all_z_error_[i], color=self.colors[i], label=self.legend_str[i])
                ax.plot(self.all_x_error_[i], self.all_z_error_[i], zdir='y', alpha=0.4, zs=5.5, color=self.colors[i]) #, linestyle='dotted', label='YZ')
                ax.plot(self.all_y_error_[i], self.all_z_error_[i], zdir='x', alpha=0.4, zs=5.5, color=self.colors[i]) #, linestyle='dotted', label='XZ')
                ax.plot(self.all_x_error_[i], self.all_y_error_[i], zdir='z', alpha=0.4, color=self.colors[i]) #, linestyle='dotted', label='XY')

            fig.legend()
            
            ax.axes.set_xlim3d(left=-5.5, right=5.5) 
            ax.axes.set_ylim3d(bottom=-5.5, top=5.5) 
            ax.axes.set_zlim3d(bottom=0, top=11) 

            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')

            # fig.set_box_aspect((np.ptp(self.all_x_error_), np.ptp(self.all_y_error_), np.ptp(self.all_z_error_)))
            # plt.axis('square')
            # ax.set_aspect('equal')
            #plt.suptitle('Inertial Derived Path During Autonomous Flight')
            plt.show()

            self.all_lock_.release()



## PLOT ALL XYZ VALS WHEN CERTAIN KEY PRESSED


    ## Key press handling

    def press(self, key):
        
        if key == 'e' or key == 'enter':

            if len(self.all_x_error_) == len(self.all_y_error_) and len(self.all_x_error_) == len(self.all_z_error_):
                if len(self.all_x_error_) < 1:
                    print("Sequence has length 0, nothing to plot")

                else:
                    self.draw_image()
            else:
                print("Data axes have different lengths")


        else:
            print("Unsupported command:", key)
        

    def keylistener(self):

        print("Press 'e/enter' to plot sequence\n")

        listen_keyboard(
            on_press=self.press,
        )





###############################################################################
# Main
###############################################################################


if __name__ == "__main__":
    rclpy.init()

    print("Starting IdDrawer node")

    minimal_publisher = IdDrawer()

    minimal_publisher.key_thread.start()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()