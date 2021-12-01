#! /usr/bin/env python
"""
File         : explore_find.py
Information  : This code file was designed to make Jackal robot navigate unknown environment while searching
               for April Tags
Author       : Mohamed Tolba
Last Modified: 27 Nov 2021
"""

import os
import time
import math
import numpy as np
import rospy
import actionlib

from frontiers_finder import find_frontiers
from apriltag_detection import AprilTagDetect
from move_robot import move_jackal

from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from finders_keepers.msg import frontier_msg
from finders_keepers.msg import ctrl_msg
from nav_msgs.srv import GetPlan, GetPlanRequest

inf = float('inf')

class jackal_explore():
    def __init__(self):
        # Constant parameters
        self.pos_tol = 0.5  # The max acceptable error in position
        self.max_cycles = 3  # The max number of cyclic rotations allowed while achieving a given goal position
                             # It helps indicate if the robot has done a lot of rotations while it is trying to achieve
                             # a given goal position
        self.d_max = 10      # The max distance allowed to be covered while achieving a given goal position without
                             # checking for new frontiers

        # Message objects
        self.goal_pose = MoveBaseGoal()
        self.ctrl_msg = ctrl_msg()
        self.get_plan_request = GetPlanRequest()

        # Objects
        self.find_frontiers = find_frontiers()
        self.find_new_tag = AprilTagDetect()
        self.move = move_jackal()

        # Initialize Publisher(s)
        self.ctrl_pub = rospy.Publisher('/ctrl', ctrl_msg, queue_size=1)

        # Initialize Subscriber(s)
        self.odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_sub_cb)
        self.odom_sub_msg = Odometry()
        self.frontiers_sub = rospy.Subscriber("/frontiers", frontier_msg, self.frontiers_sub_cb)

        # Initialize Action Client(s)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # waits until the action server is up and running
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting exploration ...")

        # Initialize Service Client(s)
        rospy.wait_for_service("/move_base/make_plan")
        self.get_plan_service = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        # Initial values
        self.odom_heading_angle_deg = 1000.0
        self.map_heading_angle_deg = 1000.0
        self.new_reading_flag = -1
        self.odom_pos = []
        self.map_pos = []
        self.current_position = []
        self.prev_position = []
        self.distance = 0.0
        self.pose_sample = []
        self.frontier_pnts = []
        self.frontier_radii = []
        self.nextpnt = []
        self.prevpnt = []
        self.GBFS_mode = 1
        self.frontier_mode = 1
        self.free_space_threshold = 0
        self.ref_point = [0,0]
        self.move_base_status = -1
        self.status = -1
        self.initial_goal_flag = 0
        self.find_new_frontiers = True
        self.send_new_frontier = True
        self.goal_position = []
        self.total_rotated_angle = 0.0
        self.prev_heading_deg = 0.0
        self.current_heading_deg = 0.0
        self.rejected_frontiers = []
        self.tag_ids = []
        stream = os.popen('rospack find finders_keepers')
        directory = stream.readline()
        self.txt_file_path = str(directory[:-1]) + "/src/tag_ids.txt"
        with open(self.txt_file_path, 'r') as f:
            for line in f:
                self.tag_ids.append(int(line))

        # Other Stuff
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz
        rospy.on_shutdown(self.shutdownhook)

    def odom_sub_cb(self, msg):
        self.new_reading_flag = 0
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

    def frontiers_sub_cb(self, msg):
        self.frontiers_sub = msg
        x = self.frontiers_sub.frontier_x
        y = self.frontiers_sub.frontier_y
        radii = self.frontiers_sub.frontier_radius
        self.frontier_pnts = []
        self.frontier_radii = []
        if len(x) > 0:
            for i in range(len(x)):
                self.frontier_pnts.append([x[i], y[i]])
                self.frontier_radii.append(radii[i])

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback_msg):
        self.new_reading_flag = 1
        x = feedback_msg.base_position.pose.position.x
        y = feedback_msg.base_position.pose.position.y
        self.map_pos = [x, y]

        z = feedback_msg.base_position.pose.orientation.z
        w = feedback_msg.base_position.pose.orientation.w
        heading_angle_rad = 2 * math.atan2(z, w)
        heading_angle_deg = heading_angle_rad * 180.0 / math.pi
        if heading_angle_deg < 0:
            heading_angle_deg += 360

        self.map_heading_angle_deg = heading_angle_deg

    def done_cb(self, status, result):
        self.move_base_status = status
        self.status = status
        if self.status == 2:
            rospy.loginfo("The current goal has been cancelled")
        elif self.status == 3:
            rospy.loginfo("Goal pose reached")
            if self.initial_goal_flag != 2:
                self.client.cancel_goal()
                self.client.wait_for_result()
                self.find_new_frontiers = True
                self.send_new_frontier = True
                rospy.loginfo("Finding new goal ...")
            self.move_base_status = -1
            self.goal_position = []
        elif self.status == 4:
            rospy.loginfo("Goal pose was aborted by the Action Server")
        elif self.status == 5:
            rospy.loginfo("Goal pose has been rejected by the Action Server")
        elif self.status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

    def shutdownhook(self):
        rospy.loginfo("Shutdown time!")
        self.client.cancel_goal()
        self.client.wait_for_result()
        self.move.stop_jackal()
        self.ctrl_c = True

    def full_rotation(self, step=90):
        no_steps = int(round(360 / step))
        turn_success = True
        for i in range(no_steps-1):
            obstacle = self.move.obstacle_detection('full_turn', 0.8)
            if turn_success == True and obstacle == False:
                rospy.loginfo("Executing full rotation motion...")
                self.find_tags()
                turn_success = self.move.turn_deg(self.odom_heading_angle_deg + step, 2)
            else:
                rospy.loginfo("Cannot make full rotation!")
                full_rotation_success = False
                return full_rotation_success
        full_rotation_success = True
        return full_rotation_success

    def initiate_motion(self, rotation=True):
        rospy.loginfo('Initiate the robot motion ..')
        full_rotation_success = False
        if rotation == True:
            full_rotation_success = self.full_rotation(60)
        if rotation == False or not full_rotation_success:
            front_obstacle = self.move.obstacle_detection('forward', 2.0)
            if not front_obstacle:
                self.move.move_linear_obs(0.5, 3, 0.9)
            else:
                back_obstacle = self.move.obstacle_detection('backward', 2.0)
                if not back_obstacle:
                    self.move.move_linear_obs(-0.5, 3, 0.9)
                else:
                    rospy.loginfo("The robot may be in very close proximity to obstacles!")
                    self.move.free_robot(1.2)

        self.find_tags()

    def explore(self):
        start_time = time.time()
        while not self.ctrl_c:
            ############################################################################################################
            # Checking for April Tags
            self.find_tags()
            ############################################################################################################
            # Initializing the robot motion
            if self.initial_goal_flag == 0:
                self.client.cancel_goal()
                self.client.wait_for_result()
                self.initiate_motion()
                self.find_tags()
                self.initial_goal_flag = 1
                self.find_new_frontiers = True
                self.send_new_frontier = True
                self.rate.sleep()
                continue
            ############################################################################################################
            # Getting the current robot pose data
            psi_deg = 0
            x = 0
            y = 0
            if len(self.map_pos) > 0:
                x = self.map_pos[0]
                y = self.map_pos[1]
                psi_deg = self.map_heading_angle_deg
                self.pose_sample.append([x, y, psi_deg])
                self.prev_position = self.current_position
                self.prev_heading_deg = self.current_heading_deg
                self.current_position = [x, y]
                self.current_heading_deg = psi_deg
                self.map_pos = []

            elif len(self.odom_pos) > 0:
                x = self.odom_pos[0]
                y = self.odom_pos[1]
                psi_deg = self.odom_heading_angle_deg
                self.odom_pos = []

            else:
                self.rate.sleep()
                continue
            ############################################################################################################
            # Checking if the robot is stuck or not
            if len(self.pose_sample) >= 30 and len(self.goal_position) > 0:
                rospy.loginfo("A 30-point sample has been collected for the robot pose")
                sample_array = np.array(self.pose_sample)
                mean_x = np.average(sample_array[:, 0])
                mean_y = np.average(sample_array[:, 1])
                # mean_psi = np.average(sample_array[:, 2])
                distance_change = euclidean_distance(self.current_position, [mean_x, mean_y])
                if distance_change < 0.1:
                    rospy.loginfo('The covered distance is less than 0.1 m over the last 30-point sample!')
                    self.client.cancel_goal()
                    self.client.wait_for_result()
                    obstacle = self.move.obstacle_detection('full_turn', 0.9)
                    if obstacle:
                        rospy.loginfo("The robot might be stuck!")
                        rospy.loginfo("Recovery behavior: Freeing the robot...")
                        self.move.free_robot(1.0)
                    else:
                        rospy.loginfo("The goal position might be unreachable!")
                        rospy.loginfo("Recovery behavior: Initiate robot motion and resetting the goal position...")
                        self.initiate_motion(False)  # Initiate motion without full rotation
                        indx = self.frontier_pnts.index(self.goal_position)
                        self.frontier_pnts.remove(self.goal_position)
                        self.frontier_radii.pop(indx)

                    self.find_new_frontiers = False
                    self.send_new_frontier = True
                    self.goal_position = []
                    self.pose_sample = []

                else:
                    self.pose_sample = []
            ############################################################################################################
            # Finding new frontiers
            if self.find_new_frontiers:
                print '###########################################################################'
                self.find_frontiers.findFrontier(False, self.frontier_mode)  # Finding new frontiers
                self.find_new_frontiers = False
                self.send_new_frontier = True
                self.goal_position = []
                time.sleep(1)
            ############################################################################################################
            # Filtering the found frontiers
            if self.send_new_frontier:
                self.nextpnt = []
                self.goal_position = []
                self.total_rotated_angle = 0.0
                self.distance = 0.0

                # Frontiers filter 1 ***********************************************************************************
                if len(self.rejected_frontiers) > 0:
                    for frontier_pnt in self.frontier_pnts:
                        for rejected_frontier_pnt in self.rejected_frontiers:
                            if euclidean_distance(frontier_pnt, rejected_frontier_pnt) <= self.pos_tol:
                                indx = self.frontier_pnts.index(frontier_pnt)
                                self.frontier_pnts.remove(frontier_pnt)
                                self.frontier_radii.pop(indx)

                # Frontiers filter 2 ***********************************************************************************
                if self.prevpnt and len(self.frontier_pnts) > 0:
                    for frontier_pnt in self.frontier_pnts:
                        if euclidean_distance(frontier_pnt, self.prevpnt) <= self.pos_tol:
                            indx = self.frontier_pnts.index(frontier_pnt)
                            self.frontier_pnts.remove(frontier_pnt)
                            self.frontier_radii.pop(indx)

                if len(self.frontier_pnts) > 0:
                    rospy.loginfo('There are frontier points')

                    # Frontiers filter 3 *******************************************************************************
                    if self.GBFS_mode == 1:
                        rospy.loginfo('Checking the closest frontier')
                        idx = self.find_closest_frontier([x, y], self.frontier_pnts)
                        if not idx:
                            idx = find_closest([x, y], self.frontier_pnts)
                        #idx = weighing_frontier([x, y], self.frontier_pnts, self.frontier_radii)
                        rospy.loginfo('The closest frontier found has radius= ' + str(self.frontier_radii[idx]) + ' m')
                        #rospy.loginfo('The next frontier has radius= ' + str(self.frontier_radii[idx]) + ' m')
                        self.nextpnt = self.frontier_pnts[idx]
                    else:
                        rospy.loginfo('Checking the biggest frontier')
                        max_frontier_size = max(self.frontier_radii)
                        idx = self.frontier_radii.index(max_frontier_size)
                        rospy.loginfo('The biggest frontier found has radius= ' + str(self.frontier_radii[idx]) + ' m')
                        self.nextpnt = self.frontier_pnts[idx]

                    # Frontiers filter 4 *******************************************************************************
                    if self.frontier_radii[idx] > 2:  # If the frontier radius is large
                        self.GBFS_mode = 1
                        self.find_new_frontiers = False
                        self.send_new_frontier = False
                    else:
                        rospy.loginfo("Checking frontier accessibility...")
                        accessible, cell_cost = self.find_frontiers.isaccessible([x, y], self.nextpnt, self.free_space_threshold, self.GBFS_mode)
                        if cell_cost == 0:
                            self.ref_point = [x,y]
                        if accessible == -1: # The accessibility cannot be checked from the current robot location (The robot location map cell has a high cost value)
                            rospy.loginfo('The robot is close to an obstacle')
                            rospy.loginfo('The frontier point accessibility cannot be checked by GBFS using the current robot location')
                            rospy.loginfo('Checking the frontier accessibility using the last zero-cost robot location...')
                            accessible, cell_cost = self.find_frontiers.isaccessible(self.ref_point, self.nextpnt, self.free_space_threshold, self.GBFS_mode) # Checking if the next point is accessible using a ref point with zero cost
                        if accessible == 1:
                            rospy.loginfo('The frontier point is accessible')
                            self.GBFS_mode = 1
                            self.find_new_frontiers = False
                            self.send_new_frontier = False
                        elif accessible == 0:
                            rospy.loginfo('The frontier point is not accessible')
                            rospy.loginfo('Finding another frontier point...')
                            if self.GBFS_mode == 2:
                                self.rejected_frontiers.append(self.nextpnt)
                            indx = self.frontier_pnts.index(self.nextpnt)
                            self.frontier_pnts.remove(self.nextpnt)
                            self.frontier_radii.pop(indx)
                            self.find_new_frontiers = False
                            self.send_new_frontier = True
                            self.nextpnt = []
                            self.goal_position = []
                            continue

                    ####################################################################################################
                    # Sending the filtered frontier as a new goal position for the ROS Navigation Stack
                    if self.nextpnt:
                        self.move.free_robot(1.1)
                        time.sleep(0.5)
                        if self.new_reading_flag == 0:
                            x = self.odom_pos[0]
                            y = self.odom_pos[1]
                            psi_deg = self.odom_heading_angle_deg
                        elif self.new_reading_flag == 1:
                            x = self.map_pos[0]
                            y = self.map_pos[1]
                            psi_deg = self.map_heading_angle_deg

                        d, new_heading_deg = self.findPlan([x,y],psi_deg,self.nextpnt,0)
                        rospy.loginfo('Rotating the robot to the best orientation for navigation...')
                        self.move.turn_deg(new_heading_deg, 1)
                        self.move.move_linear_obs(0.5,2,1.0)
                        time.sleep(0.5)

                        rospy.loginfo('Sending the new frontier to move_base node...')
                        self.goal_position = self.nextpnt
                        self.sendGoal(self.goal_position, psi_deg)
                        time.sleep(1)
                        self.find_new_frontiers = False
                        self.send_new_frontier = False
                        self.distance = 0
                        self.total_rotated_angle = 0.0
                        self.GBFS_mode = 1
                        self.prevpnt = self.nextpnt
                        self.nextpnt = []
                        self.pose_sample = []

                ########################################################################################################
                # Changing the frontier search criteria if there are not frontier points found
                else:  # If no frontier points
                    if self.GBFS_mode == 1:  # The tight accessibility check mode (Checks accessibility through free spaces only)
                        rospy.loginfo("No accessible frontier points found. Relaxing accessibility check criteria ...")
                        self.GBFS_mode = 2  # The relaxed accessibility check mode (Checks accessibility through unknown and free spaces)
                        self.find_new_frontiers = True
                        self.send_new_frontier = True
                        self.nextpnt = []
                        self.goal_position = []
                        continue
                    else:
                        rospy.loginfo("No accessible frontier points found after relaxing accessibility check criteria.")
                        self.find_new_frontiers = False
                        self.send_new_frontier = False
                        self.nextpnt = []
                        self.goal_position = []
            ############################################################################################################
            # Check the following if it is not required to send a new frontier and there is a goal position
            elif self.goal_position:
                # Checking if the robot is at or very close to the goal position ***************************************
                error_distance = euclidean_distance([x, y], self.goal_position)
                if error_distance <= self.pos_tol or self.move_base_status == 3:
                    rospy.loginfo("The error distance= " + str(round(error_distance, 2)) + " m")
                    if self.move_base_status != 3:
                        self.client.cancel_goal()
                        self.client.wait_for_result()
                    else:
                        self.move_base_status = -1
                    self.full_rotation(60)
                    self.find_new_frontiers = True
                    self.send_new_frontier = True
                    self.distance = 0.0
                    self.total_rotated_angle = 0.0
                    self.goal_position = []

                # Checking if the goal position has been aborted or rejected by the Action Server **********************
                elif self.move_base_status == 4 or self.move_base_status == 5:
                    rospy.loginfo('The frontier point is not accessible')
                    self.rejected_frontiers.append(self.goal_position)
                    indx = self.frontier_pnts.index(self.goal_position)
                    self.frontier_pnts.remove(self.goal_position)
                    self.frontier_radii.pop(indx)
                    rospy.loginfo('Frontier point removed')
                    rospy.loginfo('Finding another frontier point...')
                    self.find_new_frontiers = False
                    self.send_new_frontier = True
                    self.move_base_status = -1
                    self.distance = 0.0
                    self.total_rotated_angle = 0.0
                    self.goal_position = []

                # Checking if the robot moved too much while trying to achieve the goal position 8**********************
                elif self.prev_position and self.prev_heading_deg:
                    self.distance += euclidean_distance(self.prev_position, self.current_position)
                    self.total_rotated_angle += abs(self.current_heading_deg - self.prev_heading_deg)
                    self.prev_position = self.current_position
                    self.prev_heading_deg = self.current_heading_deg
                    number_of_cycles = int(self.total_rotated_angle / 360.0)
                    if self.distance >= self.d_max:
                        rospy.loginfo("The robot has covered long distance " + str(self.distance) + " m")
                        if self.move_base_status != 3:
                            self.client.cancel_goal()
                            self.client.wait_for_result()
                        else:
                            self.move_base_status = -1
                        self.find_new_frontiers = True
                        self.send_new_frontier = True
                        self.total_rotated_angle = 0.0
                        self.distance = 0.0
                        self.goal_position = []

                    elif number_of_cycles >= self.max_cycles:
                        rospy.loginfo("The robot is rotating too much!")
                        if self.move_base_status != 3:
                            self.client.cancel_goal()
                            self.client.wait_for_result()
                        else:
                            self.move_base_status = -1

                        self.move.free_robot(1.0)
                        indx = self.frontier_pnts.index(self.goal_position)
                        self.frontier_pnts.remove(self.goal_position)
                        self.frontier_radii.pop(indx)
                        self.find_new_frontiers = False
                        self.send_new_frontier = True
                        self.total_rotated_angle = 0.0
                        self.distance = 0.0
                        self.goal_position = []
            ############################################################################################################
            # Shutting down process if  it is not required to send new frontiers and there is no goal position
            else:
                rospy.loginfo("No more frontier points to explore!")
                print '################################################################'
                current_time = time.time()
                elapsed_time = current_time - start_time
                rospy.loginfo("The elapsed time is " + str(elapsed_time) + " seconds")

                self.ctrl_msg.explore_find_status = "no_frontiers"
                self.ctrl_pub.publish(self.ctrl_msg)

                stream = os.popen('rospack find finders_keepers')
                if stream:
                    self.tag_ids = []
                    directory = stream.readline()
                    self.txt_file_path = str(directory[:-1]) + "/src/tag_ids.txt"
                    with open(self.txt_file_path, 'r') as f:
                        for line in f:
                            self.tag_ids.append(int(line))
                self.tag_ids.sort()
                rospy.loginfo("The total number of april tags identified is " + str(
                    len(self.tag_ids)) + " with tag-id number(s): " + str(self.tag_ids))

                ans = input("Return to the start point ('Y'/'N')? ")
                if ans == 'Y':
                    rospy.loginfo("Returning to the start point ...")
                    self.move_base_status = -1
                    self.initial_goal_flag = 2
                    self.goal_position = [0, 0]
                    self.sendGoal(self.goal_position, 0)
                    self.client.wait_for_result()
                    time.sleep(1)
                    if self.status == 3:
                        rospy.loginfo("Returned Successfully to the start point")
                        ans_2 = input("Restart exploration 'E' or terminate the process 'S'?: ")
                        if ans_2 == 'E':
                            self.initial_goal_flag = 0
                            self.find_new_frontiers = True
                            self.send_new_frontier = True
                            continue
                        else:
                            rospy.signal_shutdown("Exploration finished")
                    else:
                        rospy.loginfo("Couldn't return to the start point")
                        rospy.signal_shutdown("Exploration finished")
                else:
                    rospy.signal_shutdown("Exploration finished")

    def find_closest_frontier(self, robot_position=[], frontier_pnts=[]):
        d_min = 1000.0
        closest_frontier_pnt = []
        for frontier_pnt in frontier_pnts:
            d1 = euclidean_distance(robot_position, frontier_pnt)
            if self.pos_tol < d1 < self.d_max:
                d, angle = self.findPlan(robot_position, 0, frontier_pnt, 0)
                if d < d_min and d < self.d_max:
                    d_min = d
                    closest_frontier_pnt = frontier_pnt
                else:
                    continue
            else:
                continue

        if closest_frontier_pnt:
            rospy.loginfo("The closest frontier is at distance= " + str(round(d_min, 2)) + " m")
            # print("min_path_length=", d_min)
            idx = frontier_pnts.index(closest_frontier_pnt)
            return idx

    def findPlan(self, start_position, start_orientation_deg,  goal_position, goal_orientation_deg):
        q_start = quaternion_from_euler(0, 0, start_orientation_deg * math.pi / 180, 'sxyz')
        self.get_plan_request.start.header.frame_id = "map"
        self.get_plan_request.start.pose.position.x = start_position[0]
        self.get_plan_request.start.pose.position.y = start_position[1]
        self.get_plan_request.start.pose.orientation.x = q_start[0]
        self.get_plan_request.start.pose.orientation.y = q_start[1]
        self.get_plan_request.start.pose.orientation.z = q_start[2]
        self.get_plan_request.start.pose.orientation.w = q_start[3]

        q_goal = quaternion_from_euler(0, 0, goal_orientation_deg * math.pi / 180, 'sxyz')
        self.get_plan_request.goal.header.frame_id = "map"
        self.get_plan_request.goal.pose.position.x = goal_position[0]
        self.get_plan_request.goal.pose.position.y = goal_position[1]
        self.get_plan_request.goal.pose.orientation.x = q_goal[0]
        self.get_plan_request.goal.pose.orientation.y = q_goal[1]
        self.get_plan_request.goal.pose.orientation.z = q_goal[2]
        self.get_plan_request.goal.pose.orientation.w = q_goal[3]

        self.get_plan_request.tolerance = self.pos_tol #If you can not reach the target, the latest available to the constraint

        get_plan_response = self.get_plan_service(self.get_plan_request)

        d = 0.0
        for i in range(len(get_plan_response.plan.poses)-1):
            x1 = get_plan_response.plan.poses[i].pose.position.x
            y1 = get_plan_response.plan.poses[i].pose.position.y
            x2 = get_plan_response.plan.poses[i+1].pose.position.x
            y2 = get_plan_response.plan.poses[i+1].pose.position.y
            pnt1 = [x1,y1]
            pnt2 = [x2,y2]
            d += euclidean_distance(pnt1, pnt2)

        X_start = start_position[0]
        Y_start = start_position[1]
        X_goal = goal_position[0]
        Y_goal = goal_position[1]
        for i in range(len(get_plan_response.plan.poses)):
            X = get_plan_response.plan.poses[i].pose.position.x
            Y = get_plan_response.plan.poses[i].pose.position.y
            d_check = euclidean_distance([X_start,Y_start],[X,Y])
            if d_check >= 0.2:
                X_goal = X
                Y_goal = Y
                break
        new_heading_deg = math.atan2((Y_goal - Y_start), (X_goal - X_start)) * 180.0 / math.pi  # deg
        return [d,new_heading_deg]




    def sendGoal(self, goal_position, goal_orientation_deg):
        q = quaternion_from_euler(0, 0, goal_orientation_deg * math.pi / 180, 'sxyz')
        self.goal_pose.target_pose.header.stamp = rospy.Time.now()
        self.goal_pose.target_pose.header.frame_id = "map"
        self.goal_pose.target_pose.pose.position.x = goal_position[0]
        self.goal_pose.target_pose.pose.position.y = goal_position[1]
        self.goal_pose.target_pose.pose.orientation.x = q[0]
        self.goal_pose.target_pose.pose.orientation.y = q[1]
        self.goal_pose.target_pose.pose.orientation.z = q[2]
        self.goal_pose.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(self.goal_pose, self.done_cb, self.active_cb, self.feedback_cb)

    def find_tags(self):
        tag_id = self.find_new_tag.find_apriltag(False)
        if tag_id > -1:
            rospy.loginfo('New April Tag has been found')
            self.tag_ids.append(tag_id)
            rospy.loginfo(
                "The number of tags identified so far is " + str(len(self.tag_ids)) + " with tag-id number(s) " + str(
                    self.tag_ids))


def euclidean_distance(pnt1, pnt2):
    x1 = pnt1[0]
    y1 = pnt1[1]
    x2 = pnt2[0]
    y2 = pnt2[1]
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def find_closest(node, nodes):
    nodes = np.asarray(nodes)
    node = np.asarray(node)
    dist_2 = np.sum((nodes - node) ** 2, axis=1)
    return np.argmin(dist_2)

def weighing_frontier(robot_position, frontier_pnts, frontier_radii):
    i = 0
    weight = []
    for frontier_pnt in frontier_pnts:
        distance = euclidean_distance(robot_position, frontier_pnt)
        radius = frontier_radii[i]
        weight.append(radius/0.9/distance)
        i = i+1
    weight = np.asarray(weight)
    return np.argmax(weight)

def main():
    rospy.init_node('explore_find')
    rospy.sleep(1)
    gc = jackal_explore()
    rospy.sleep(1)
    gc.explore()
    try:
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print("Shutting down")

if __name__ == "__main__":
    main()
