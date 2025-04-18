#!/usr/bin/env python3

#================================================================
# File name: vicon_tracker_pp.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun vicon_control vicon_tracker_pp.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal


# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker



class PurePursuit(object):
    
    def __init__(self):
        
        # 0.5 - 0.1 - 0.41

        self.rate = rospy.Rate(50)

        self.look_ahead = 0.5 # 4
        self.wheelbase  = 0.325 # meters
        self.offset     = 0.15 # meters        
        
        self.ctrl_pub = rospy.Publisher("/car_1/offboard/command", AckermannDrive, queue_size=1)
        self.drive_msg = AckermannDrive()
        # self.drive_msg.header.frame_id = "f1tenth_control"
        self.drive_msg.speed     = 0.2 # m/s, reference speed

        self.vicon_sub = rospy.Subscriber('/car_state', Float64MultiArray, self.carstate_callback)
        # self.odom_sub = rospy.Subscriber('/car_1/base/odom', Odometry, self.odom_callback)
        self.gt_sub = rospy.Subscriber('/car_1/ground_truth', Odometry, self.ground_truth_callback)

        self.marker_pub = rospy.Publisher("/path_marker", Marker, queue_size=1)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        
        # read waypoints into the system 
        self.goal = 0            
        self.read_waypoints() 
        self.path_points_x = np.array(self.path_points_x_record)
        self.path_points_y = np.array(self.path_points_y_record)

        self.last_goal_index = 0
        self.kp = 0.1  # Proportional gain
        self.ki = 0.0  # Integral gain
        self.kd = 0.1  # Derivative gain
        self.min_lookahead = 1
        self.max_lookahead = 2.0
        self.ct_integral = 0.0
        self.last_ct_error = 0.0
        self.last_time = None
    
    def publish_markers(self, goal_point):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns   = "pure_pursuit"
        marker.id   = 0
        marker.action = Marker.ADD
        marker.type   = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = goal_point[0]
        marker.pose.position.y = goal_point[1]
        self.marker_pub.publish(marker)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def ground_truth_callback(self, msg):
        # pull out the pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # convert quaternion → yaw (radians)
        yaw_rad = self.quaternion_to_yaw(q)

        # save into your state (you’ll convert to degrees if you like)
        self.x   = x
        self.y   = y
        self.yaw = math.degrees(yaw_rad)

        # (optional) print for debugging
        print(f"[GT CALLBACK] x: {self.x:.3f}, y: {self.y:.3f}, yaw(deg): {self.yaw:.2f}")

        
    def odom_callback(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)  # radians
        self.yaw = math.degrees(self.yaw)            # convert to degrees to match original code
        print(f"[ODOM CALLBACK] x: {self.x:.3f}, y: {self.y:.3f}, yaw: {self.yaw:.2f}")

    def carstate_callback(self, carstate_msg):
        self.x   = carstate_msg.data[0] # meters
        self.y   = carstate_msg.data[1] # meters
        self.yaw = carstate_msg.data[3] # degrees

    def read_waypoints(self):
        # read recorded GPS lat, lon, heading
        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/xyhead_demo_pp.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        # x towards East and y towards North
        self.path_points_x_record   = [float(point[0]) for point in path_points] # x
        self.path_points_y_record   = [float(point[1]) for point in path_points] # y
        self.path_points_yaw_record = [float(point[2]) for point in path_points] # yaw
        self.wp_size                = len(self.path_points_x_record)
        self.dist_arr               = np.zeros(self.wp_size)

    def get_f1tenth_state(self):

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        curr_yaw = np.radians(self.yaw)

        # reference point is located at the center of rear axle
        curr_x = self.x - self.offset * np.cos(curr_yaw)
        curr_y = self.y - self.offset * np.sin(curr_yaw)
        print(f"[GET STATE] Rear-axle adjusted pos: x={curr_x:.3f}, y={curr_y:.3f}, yaw(rad)={curr_yaw:.2f}")
        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def find_goal_index(self):
        # Calculate distances to all points
        dx = [x - self.x for x in self.path_points_x]
        dy = [y - self.y for y in self.path_points_y]
        distances = np.hypot(dx, dy)
        closest_idx = np.argmin(distances)

        # Look ahead along the path
        total_dist = 0.0
        goal_idx = closest_idx
        while goal_idx < len(self.path_points_x)-1 and total_dist < self.look_ahead:
            total_dist += self.dist((self.path_points_x[goal_idx], self.path_points_y[goal_idx]),
                                    (self.path_points_x[goal_idx+1], self.path_points_y[goal_idx+1]))
            goal_idx += 1
        return min(goal_idx, len(self.path_points_x)-1)
    def start_pp(self):
        
        while not rospy.is_shutdown():
            


            if self.goal >= len(self.path_points_x) - 2:
                self.drive_msg.speed = 0.0
                self.ctrl_pub.publish(self.drive_msg)
                rospy.loginfo("Path completed!")
                break

            # Dynamic lookahead based on speed
            self.look_ahead = np.clip(1.5 * abs(self.drive_msg.speed), 
                                 self.min_lookahead, 
                                 self.max_lookahead)


            curr_x, curr_y, curr_yaw = self.get_f1tenth_state()

            # finding the distance of each way point from the current position
            for i in range(len(self.path_points_x)):
                self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))
                # print(f"[LENGTHS X AND Y] iX: {self.path_points_x[i]}, iY: {self.path_points_y[i]}, currX: {curr_x}, currY: {curr_y}")
            print(f"[DISTANCES] First 5 distances: {self.dist_arr[:5]}")
            print(f"[CURRENT] curr_x: {curr_x}, curr_y: {curr_y}, look_ahead: {self.look_ahead}")
            # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
            self.goal = self.find_goal_index()
            print(f"[GOAL SELECTION] Candidates within lookahead: {self.goal}")

            # self.goal = None
            # for idx in reversed(goal_arr):  # start from farthest
            #     v1 = [self.path_points_x[idx] - curr_x, self.path_points_y[idx] - curr_y]
            #     v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
            #     angle = self.find_angle(v1, v2)
            #     if abs(angle) < np.pi / 2:  # means it's in front of car
            #         self.goal = idx
            #         break

            # if self.goal is None:
            #     print("[WARNING] No suitable goal in front of the car.")
            #     self.rate.sleep()
            #     continue

            # After finding goal:
            self.publish_markers((self.path_points_x[self.goal], 
                                self.path_points_y[self.goal]))

            # if len(goal_arr) > 0:
            #     self.goal = goal_arr[0]
            # else:
            #     print("[WARNING] No goal point found within lookahead window.")
            #     self.rate.sleep()
            #     continue



            # if len(goal_arr) == 0:
            #     print("[WARNING] No valid goal point found! Skipping this loop.\n")
            #     self.rate.sleep()
            #     continue



            # finding the goal point which is the last in the set of points less than the lookahead distance


            # for idx in goal_arr:
            #     v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
            #     v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
            #     temp_angle = self.find_angle(v1,v2)
            #     # find correct look-ahead point by using heading information
            #     if abs(temp_angle) < np.pi/2:
            #         self.goal = idx
            #         break




            # finding the distance between the goal point and the vehicle
            # true look-ahead distance between a waypoint and current position
            L = self.dist_arr[self.goal]
            print(f"[DISTANCE] L: {L:.3f}")

            # Vector from vehicle to goal point
            dx = self.path_points_x[self.goal] - curr_x 
            dy = self.path_points_y[self.goal] - curr_y 
                
            # Path-vehicle angle calculation    
            alpha = math.atan2(dy, dx) - curr_yaw   
                
            current_time = rospy.get_time()
            if self.last_time is None:
                dt = 1.0 / 50.0
            else:
                dt = current_time - self.last_time
            self.last_time = current_time

            # PID terms
            ct_error = math.sin(alpha) * L
            self.ct_integral += ct_error * dt
            derivative = (ct_error - self.last_ct_error) / dt if dt > 0 else 0

            # PID steering adjustment
            pid_correction = (self.kp * ct_error + 
                            self.ki * self.ct_integral + 
                            self.kd * derivative)

            # Clamp integral term to prevent windup
            self.ct_integral = np.clip(self.ct_integral, -1.0, 1.0)
            self.last_ct_error = ct_error

            # Pure Pursuit base angle
            curvature = (2 * math.sin(alpha)) / L
            base_angle = math.atan(self.wheelbase * curvature)

            # Final steering angle
            steering_angle = base_angle + pid_correction
            steering_angle = np.clip(steering_angle, -0.3, 0.3) 


            # find the curvature and the angle 
            # alpha = np.radians(self.path_points_yaw_record[self.goal]) - curr_yaw

            # # ----------------- tuning this part as needed -----------------
            # k       = 0.2
            # angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L) 
            # angle   = angle_i*2
            # ----------------- tuning this part as needed -----------------

            # f_delta = round(np.clip(angle, -0.3, 0.3), 3)

            # f_delta_deg = round(np.degrees(f_delta))

            print("Current index: " + str(self.goal))
            # ct_error = round(np.sin(alpha) * L, 3)
            print("Crosstrack Error: " + str(ct_error))
            print("Front steering angle: " + str(steering_angle) + " degrees")
            curr_x, curr_y, curr_yaw = self.get_f1tenth_state()
            print("Current State: x =", curr_x, "y =", curr_y, "yaw =", curr_yaw)
            print("\n")

            # self.drive_msg.header.stamp = rospy.get_rostime()
            self.drive_msg.steering_angle = steering_angle
            self.ctrl_pub.publish(self.drive_msg)
            print(f"[COMMAND] Steering angle: {self.drive_msg.steering_angle:.3f}, Speed: {self.drive_msg.speed:.3f}")
            self.rate.sleep()


def pure_pursuit():

    rospy.init_node('vicon_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


