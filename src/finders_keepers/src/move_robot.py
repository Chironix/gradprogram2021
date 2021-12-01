#!/usr/bin/env python
"""
File         : move_robot.py
Information  : This code file provides basic functions for robot unplanned navigation and obstacle avoidance
Author       : Mohamed Tolba
Last Modified: 27 Nov 2021
"""

import math
import time
import rospy
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

inf = float('inf')


class move_jackal():
    def __init__(self):
        # Constant parameters

        self.max_lin_vel = 1.5  # Required Maximum linear velocity (m/s) - 0.5
        self.max_turn_vel = 2.5  # Required Minimum turn velocity (rad/s) - 0.2
        self.robot_width = 0.45  # The width of jackal in meter (0.43 + 0.02 safety)
        self.robot_length = 0.53  # The length of jackal in meter (0.508 + 0.022 safety)
        self.r = 0.5 * math.sqrt(self.robot_length ** 2 + self.robot_width ** 2)
        # self.max_laser_FOV = int(round(math.acos(1 - self.robot_width ** 2 / 2 / self.r ** 2) * 180.0 / math.pi)) # 78.12 deg
        self.max_laser_FOV = 90  # The angle between the right and left laser beams intersection with the robot edges
        self.min_laser_FOV = 30

        # Message objects
        self.scan_sub_msg = LaserScan()
        self.odom_sub_msg = Odometry()

        # Initialize Publisher(s)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()

        # Initialize subscribers
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_sub_cb)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_sub_cb)

        # Initial values
        self.odom_heading_angle_deg = 1000.0
        self.odom_pos = []
        self.laser_all = []
        self.laser_all_arranged = []

        # Other Stuff
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz

    def scan_sub_cb(self, msg):
        self.scan_sub_msg = msg
        self.laser_all = self.scan_sub_msg.ranges
        part_1 = [distance for distance in self.laser_all[180:360]]
        part_2 = [distance for distance in self.laser_all[0:180]]
        self.laser_all_arranged = part_1 + part_2

    def odom_sub_cb(self, msg):
        self.odom_sub_msg = msg
        x = self.odom_sub_msg.pose.pose.position.x
        y = self.odom_sub_msg.pose.pose.position.y
        self.odom_pos = [x, y]

        z = self.odom_sub_msg.pose.pose.orientation.z
        w = self.odom_sub_msg.pose.pose.orientation.w
        heading_angle_rad = 2 * math.atan2(z, w)
        heading_angle_deg = heading_angle_rad * 180.0 / math.pi
        if heading_angle_deg < 0:
            heading_angle_deg += 360

        self.odom_heading_angle_deg = heading_angle_deg

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.cmd_pub.get_num_connections()
            if connections > 0:
                self.cmd_pub.publish(self.cmd)
                # rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def stop_jackal(self, msg=True):
        if msg:
            rospy.loginfo("Stopping the robot!")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def turn_deg(self, req_angle_deg=0.0, heading_error_tol=0.2):
        heading_error_deg_max = 1.0

        if abs(req_angle_deg) > 360.0:  # To keep the required angle below 360.0 deg
            req_angle_deg = req_angle_deg - np.sign(req_angle_deg) * math.floor(abs(req_angle_deg) / 360) * 360

        if req_angle_deg < 0.0:  # To avoid negative required angles
            req_angle_deg = req_angle_deg + 360.0

        if 359 <= req_angle_deg <= 360.0:  # To avoid confusion between 360 and 0 degrees
            req_angle_deg = 0

        self.stop_jackal(False)
        while True:
            if self.odom_heading_angle_deg <= 360.0:

                heading_error_deg = req_angle_deg - self.odom_heading_angle_deg

                if abs(heading_error_deg) > 180.0:  # To make the robot take the shorter path in turning
                    heading_error_deg = - np.sign(heading_error_deg) * (abs(heading_error_deg) - 180.0)

                if abs(heading_error_deg) > heading_error_deg_max:
                    heading_error_deg_max = abs(heading_error_deg)

                heading_error_rad_max = heading_error_deg_max * math.pi / 180.0
                gain = self.max_turn_vel / heading_error_rad_max
                if abs(heading_error_deg) > heading_error_tol:
                    heading_error_rad = heading_error_deg * math.pi / 180.0
                    angular_speed_rad = gain * (heading_error_rad) + np.sign(heading_error_rad) * 0.5

                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = angular_speed_rad
                    self.cmd_pub.publish(self.cmd)
                else:
                    self.stop_jackal(False)
                    turn_success = True
                    return turn_success
            else:
                rospy.loginfo("Incorrect heading angle reading (" + str(self.odom_heading_angle_deg) + " deg)")
                self.rate.sleep()

    def obstacle_detection(self, dir='forward', norm_tol=1.2):
        # laser_FOV = int(round(math.acos(1 - self.robot_width ** 2 / 2 / tol ** 2) * 180.0 / math.pi)) # deg
        # tol = tol*math.cos(laser_FOV*math.pi/180.0/2)
        obstacle = False
        if dir == 'forward' and len(self.laser_all_arranged) > 1:
            obstacle = self.obstacle_detection_angle(0, norm_tol)

        elif dir == 'backward' and len(self.laser_all_arranged) > 1:
            obstacle = self.obstacle_detection_angle(180, norm_tol)

        elif dir == 'full_turn' and len(self.laser_all_arranged) > 1:
            d_min = min(self.laser_all_arranged)
            d_max = max(self.laser_all_arranged)
            if d_min >= norm_tol and d_max != inf:
                obstacle = False
            else:
                obstacle = True

        return obstacle

    def obstacle_detection_angle(self, angle=0, norm_tol=1.2):
        # laser_tol = math.sqrt(norm_tol**2 + (self.robot_width/2)**2)
        # laser_FOV = int(round(math.acos(1 - self.robot_width ** 2 / 2 / laser_tol ** 2) * 180.0 / math.pi)) # deg
        d_min = 1000
        d_max = 0
        for FOV in range(2, self.max_laser_FOV, 2):
            d_right_edge, d_center, d_left_edge = self.check_obstacle_distance(angle, FOV)
            d_list = [d_right_edge, d_center, d_left_edge]
            if min(d_list) < d_min:
                d_min = min(d_list)
            if max(d_list) > d_max:
                d_max = max(d_list)

        if d_min >= norm_tol and d_max != inf:
            obstacle = False
        else:
            obstacle = True

        return obstacle

    def move_linear_obs(self, linear_speed=0.5, duration=10, norm_tol=1.2):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = 0.0
        motion_success = True
        obstacle = False
        i = 0
        while i < duration:
            if linear_speed > 0.0:  # Forward Motion Direction
                obstacle = self.obstacle_detection('forward', norm_tol)
            elif linear_speed < 0.0:  # Backward Motion Direction
                obstacle = self.obstacle_detection('backward', norm_tol)

            if obstacle == False:
                rospy.loginfo("Moving the robot with linear_vel= " + str(linear_speed) + " m/s")
                self.cmd_pub.publish(self.cmd)
                self.rate.sleep()
            else:
                motion_success = False
                self.stop_jackal()
                return motion_success
            i += 1

        return motion_success

    def move_linear(self, linear_speed=0.5, duration=10):
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = 0.0

        i=0
        while i < duration:
            rospy.loginfo("Moving the robot with linear_vel= " + str(linear_speed) + " m/s")
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()
            i += 1

        self.stop_jackal()

    def free_robot(self, req_norm_tol=1.0):
        no_attempts = 150
        no_steps = 1           # The linear motion number of steps per command
        min_tol = 0.8
        norm_tol = min_tol - 0.1

        while norm_tol < req_norm_tol:
            cnt = 1
            linear_motion_cnt = 0  # How many times the linear motion command is allowed to be executed without checking
            # for new orientation
            check_rotation = True  # Whether or not to check for a better orientation for the robot
            motion_dir = 'forward'
            rejected_j = []
            norm_tol = norm_tol + 0.1

            laser_tol = math.sqrt(norm_tol**2 + (self.robot_width/2)**2)
            laser_FOV = int(round(math.acos(1 - self.robot_width ** 2 / 2 / laser_tol ** 2) * 180.0 / math.pi))

            # if laser_FOV % 2 != 0:
            #     laser_FOV += 1

            stuck = self.obstacle_detection("full_turn", norm_tol)
            free_robot = not stuck
            if free_robot and norm_tol >= req_norm_tol:
                rospy.loginfo("The robot is free at the required tolerance " + str(req_norm_tol) + " m")
                return free_robot

            while not free_robot:
                rospy.loginfo("Attempting to free the robot ... " + str(cnt) + "/" + str(no_attempts))
                if check_rotation:
                    time.sleep(0.5)

                    check_rotation = False
                    linear_motion_cnt = 0
                    d_max = 0.0
                    Area_max = 0.0
                    i_max = -1

                    rejected_j = list(dict.fromkeys(rejected_j))  # To remove duplicates in rejected_j list
                    # Checking all lasers
                    for j in range(360):
                        if j in rejected_j:
                            continue
                        else:
                            obstacle = self.obstacle_detection_angle(j, 0.9) # To avoid collision
                            if not obstacle:
                                for FOV in range(laser_FOV, self.max_laser_FOV, 2):
                                    d_right_edge, d, d_left_edge = self.check_obstacle_distance(j, FOV)
                                    FOV_laser_tol = self.robot_width/2/math.sin(FOV*math.pi/180.0/2)
                                    FOV_norm_tol = FOV_laser_tol*math.cos(FOV*math.pi/180.0/2)
                                    if d >= FOV_norm_tol and d_right_edge >= FOV_norm_tol and d_left_edge >= FOV_norm_tol:
                                        if d != inf and d_right_edge != inf and d_left_edge != inf:
                                            continue
                                        else:
                                            rejected_j.append(j)
                                            continue
                                    else:
                                        rejected_j.append(j)
                                        continue
                            else:
                                rejected_j.append(j)
                                continue

                    for j in range(360):
                        if j in rejected_j:
                            continue
                        else:
                                d_right_edge, d, d_left_edge = self.check_obstacle_distance(j, laser_FOV)
                                if d > norm_tol and d_right_edge > norm_tol and d_left_edge > norm_tol:
                                    if d != inf and d_right_edge != inf and d_left_edge != inf:
                                        # Calculating the area of the triangle that has two edges d_right_edge and
                                        # d_left_edge with laser_FOV angle in between
                                        b = d_right_edge
                                        c = d_left_edge
                                        a = math.sqrt(b**2 + c**2 - 2*b*c*math.cos(laser_FOV*math.pi/180.0))
                                        s = (a+b+c)/2 # Semi-perimeter of the triangle
                                        Area = math.sqrt(s*(s-a)*(s-b)*(s-c)) # Enclosed area of the triangle
                                        if Area > Area_max:
                                            Area_max = Area
                                            req_delta_heading_deg = j
                                            j_max = j
                                            d_max = d
                    if d_max > 0:
                        if (0 <= j_max <= 90) or (270 <= j_max <= 359):
                            motion_dir = 'forward'
                        else:
                            motion_dir = 'backward'
                            req_delta_heading_deg = req_delta_heading_deg - 180.0

                        req_heading_deg = self.odom_heading_angle_deg + req_delta_heading_deg
                    else:
                        rospy.loginfo("The robot is in very tight situation")
                        norm_tol = min_tol
                        laser_tol = math.sqrt(norm_tol ** 2 + (self.robot_width / 2) ** 2)
                        laser_FOV = int(round(math.acos(1 - self.robot_width ** 2 / 2 / laser_tol ** 2) * 180.0 / math.pi))

                        j_max = -1
                        d2_max = 0.0
                        for i in range(360):
                            d2_right_edge, d2, d2_left_edge = self.check_obstacle_distance(i, self.min_laser_FOV)
                            if d2 != inf and d2_right_edge != inf and d2_left_edge != inf:
                                if d2 - d2_max > 0.5:
                                    d2_max = d2
                                    req_delta_heading_deg = i
                                    i_max = i
                        if d2_max > 0:
                            rejected_j = []
                            if (0 <= i_max <= 90) or (270 <= i_max <= 359):
                                motion_dir = 'forward'
                            else:
                                motion_dir = 'backward'
                                req_delta_heading_deg = req_delta_heading_deg - 180.0
                            req_heading_deg = self.odom_heading_angle_deg + req_delta_heading_deg
                        else:
                            cnt = no_attempts
                            req_delta_heading_deg = 0
                            req_heading_deg = self.odom_heading_angle_deg + req_delta_heading_deg

                    # For printing purposes (The big angles and negative angles are also handled in turn_deg function):
                    if req_heading_deg > 359:  # To keep the angle below 360.0 deg
                        req_heading_deg = req_heading_deg - 360
                    elif req_heading_deg < 0:  # To avoid negative angles
                        req_heading_deg = req_heading_deg + 360

                    rospy.loginfo("Required req_delta_heading_deg= " + str(req_delta_heading_deg) + " deg")
                    rospy.loginfo("Turning to the new heading angle= " + str(req_heading_deg) + " deg")

                    self.turn_deg(req_heading_deg, 0.5)

                    rospy.loginfo("The motion direction is " + str(motion_dir))

                if motion_dir == 'forward':
                    if i_max > -1:
                        self.move_linear(0.5, 1)
                        d2_right_edge, d2, d2_left_edge = self.check_obstacle_distance(0, self.min_laser_FOV)
                        if d2 != inf and d2_right_edge != inf and d2_left_edge != inf:
                            linear_motion_cnt += 0.5
                        else:
                            check_rotation = True
                    else:
                        front_motion_success = self.move_linear_obs(0.5, no_steps, norm_tol)
                        if front_motion_success == False:
                            rejected_j.append(j_max)
                            check_rotation = True
                        else:
                            linear_motion_cnt += 1
                            rejected_j = []

                elif motion_dir == 'backward':
                    if i_max > -1:
                        self.move_linear(-0.5, 1)
                        d2_right_edge, d2, d2_left_edge = self.check_obstacle_distance(180, self.min_laser_FOV)
                        if d2 != inf and d2_right_edge != inf and d2_left_edge != inf:
                            linear_motion_cnt += 0.5
                        else:
                            check_rotation = True
                    else:
                        back_motion_success = self.move_linear_obs(-0.5, no_steps, norm_tol)
                        if back_motion_success == False:
                            rejected_j.append(j_max)
                            check_rotation = True
                        else:
                            linear_motion_cnt += 1
                            rejected_j = []

                if linear_motion_cnt >= 25:
                    rejected_j = []
                    check_rotation = True

                stuck = self.obstacle_detection("full_turn", norm_tol)
                if not stuck:
                    rospy.loginfo("The robot is now free with tolerance " + str(norm_tol) + " m")
                    if norm_tol >= req_norm_tol:
                        free_robot = True
                        rospy.loginfo("The robot is free at the required tolerance " + str(req_norm_tol) + " m")
                        return free_robot
                    else:
                        break
                elif cnt < no_attempts:
                    free_robot = False
                    cnt += 1
                    continue
                elif norm_tol > min_tol:
                    free_robot = False
                    rospy.loginfo("Freeing the robot may need more than " + str(cnt) + " attempts")
                    ans = input("Do you want to continue trying ('Y'/'N')? ")
                    if ans == 'Y':
                        cnt = 1
                        continue
                    else:
                        return free_robot
                else:
                    free_robot = False
                    rospy.loginfo("Cannot free the robot!")
                    rospy.loginfo("Needs human intervention")

                    while not free_robot:
                        ans = input(
                            "If you want the robot to retry to free itself, write 'try', otherwise the program will be terminated: ")
                        if ans == 'try':
                            stuck = self.obstacle_detection("full_turn", norm_tol)
                            free_robot = not stuck
                            if free_robot:
                                rospy.loginfo("The robot has been freed successfully with tolerance " + str(norm_tol) + " m")
                                break
                                # return free_robot
                            else:
                                rospy.loginfo("The robot is still not free")
                                rospy.loginfo("Retrying the free_robot algo.")
                                cnt = 1
                                break
                        else:
                            rospy.signal_shutdown("Stuck robot")

    def check_obstacle_distance(self, angle, laser_FOV = 90):
        # "angle" is an integer angular value [deg]
        if angle > 359:  # To keep the angle below 360.0 deg
            angle = angle - 360
        elif angle < 0:  # To avoid negative angles
            angle = angle + 360

        theta_right_edge = angle - laser_FOV / 2
        if theta_right_edge > 359:  # To keep the angle below 360.0 deg
            theta_right_edge = theta_right_edge - 360
        elif theta_right_edge < 0:  # To avoid negative angles
            theta_right_edge = theta_right_edge + 360

        theta_left_edge = angle + laser_FOV / 2
        if theta_left_edge > 359:  # To keep the angle below 360.0 deg
            theta_left_edge = theta_left_edge - 360
        elif theta_left_edge < 0:  # To avoid negative angles
            theta_left_edge = theta_left_edge + 360

        d_right_edge = self.laser_all_arranged[theta_right_edge] * math.cos(laser_FOV*math.pi/180.0/2)
        d_left_edge = self.laser_all_arranged[theta_left_edge] * math.cos(laser_FOV*math.pi/180.0/2)
        d_center = self.laser_all_arranged[angle]

        return [d_right_edge, d_center, d_left_edge]

def main():
    rospy.init_node('move_jackal')  # make node
    move = move_jackal()
    rospy.sleep(1)
    # move.turn_deg(180,1)
    move.free_robot(1.2)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
