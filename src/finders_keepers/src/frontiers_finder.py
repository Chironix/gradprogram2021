#!/usr/bin/env python
"""
File         : explore_find.py
Information  : This code file finds unexplored frontiers in a given global costmap
               It also checks the accessibility of a given frontier given a robot location using
               Greedy Best First Search Algorithm
Author       : Mohamed Tolba
Last Modified: 25 Nov 2021
Credit       : The basic idea and a part of the algorithm was taken from Michael Wiznitzer work on frontier exploration
               using Jackal robot (https://mechwiz.github.io/Portfolio/jackal.html)
"""
import time
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

from finders_keepers.msg import frontier_msg
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

from Greedy_Best_First_Search import GBFS

# Important parameters
min_frontier_size = 0.4  # The minimum size of frontier to be accepted as a frontier. It is the minimum acceptable
# radius (in meter) of a circle enclosing the frontier.
show_frontiers = True  # Whether or not to plot the map grayscale picture with the found frontiers on it while running.


class find_frontiers():
    def __init__(self):
        # Parameters
        self.footprint_xy = [[0.254, 0.215], [0.254, -0.215], [-0.254, -0.215],
                             [-0.254, 0.215]]  # The jackal robot footprint
        # Message objects
        self.costmap = OccupancyGrid()
        self.grid = OccupancyGrid()
        self.cost_update = OccupancyGridUpdate()
        self.frontiers = frontier_msg()

        # Objects
        self.GBFS_obj = GBFS()

        # Initialize Publisher(s)
        self.frontiers_pub = rospy.Publisher('/frontiers', frontier_msg, queue_size=1)

        # Initialize Subscriber(s)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_sub_cb, queue_size=1)
        self.global_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.gcost_sub_cb,
                                           queue_size=1)
        self.globalup_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate,
                                             self.gcostup_sub_cb, queue_size=1)

        # Initial values
        self.flagm = 0
        self.flagg = 0
        self.init = 0
        self.newcost = []
        self.pic_mat = []

        # Other Stuff
        self.ctrl_c = False
        self.rate = rospy.Rate(10)  # 10hz
        rospy.on_shutdown(self.shutdownhook)

    def gcost_sub_cb(self, data):  # Callback function on receiving a global costmap
        self.newcost = []
        self.flagm = 1
        self.costmap = data

        width = self.costmap.info.width
        height = self.costmap.info.height
        pnts = self.costmap.data

        # To convert the vector pnts into matrix newcost of size height x width
        cnt = 0
        for x in range(height):
            newcost_x = []
            for y in range(width):
                newcost_x.append(pnts[cnt])
                cnt += 1
            self.newcost.append(newcost_x)

    def map_sub_cb(self, data):  # Callback function on receiving a map
        self.grid = data

        if self.flagm == 1:
            self.init = 1
            self.flagm = 0

    def gcostup_sub_cb(self, data):
        self.flagg = 1
        self.cost_update = data

        if self.init == 1 and self.flagm == 0:
            gpnts = self.cost_update.data  # Column vector (global costmap update data)
            gwidth = self.cost_update.width
            gheight = self.cost_update.height
            gx = self.cost_update.x
            gy = self.cost_update.y

            # To convert the vector gpnts into matrix newcost of size height x width
            cnt = 0
            for x in range(gheight):
                for y in range(gwidth):
                    g = gpnts[cnt]
                    self.newcost[x + gy][y + gx] = g
                    cnt += 1

    def findFrontier(self, continous_running=False, frontier_mode=1):
        while not self.ctrl_c:
            if self.flagg == 1 and self.init == 1:
                self.flagg = 0
                rospy.loginfo('Received a new map, global costmap and global costmap update')
                rospy.loginfo('Finding new frontiers...')

                map_resolution = self.grid.info.resolution
                width = self.grid.info.width
                height = self.grid.info.height
                ox = self.grid.info.origin.position.x
                oy = self.grid.info.origin.position.y
                pnts = self.grid.data  # Map data
                gpnts = np.array(self.newcost).flatten().tolist()  # Global costmap (or global costmap update) data

                ####################################################################################################
                ## Converting the map data into grayscale picture, where white color represents free space,
                # gray color represents unknown space, and black color represents occupied space.
                # In map data: 0 -> free space, -1 -> unknown space, and 100 -> occupied space.
                # In grayscal pic: 0 -> black, 50 -> grey, and 100 -> white
                if frontier_mode == 1:
                    gmin = 0
                elif frontier_mode == 2:
                    gmin = 10
                cnt = 0
                self.pic_mat = []
                for x in range(height):
                    pic_x = []
                    for y in range(width):
                        p = pnts[cnt]
                        g = gpnts[cnt]

                        if (g > gmin):
                            p = 0
                        else:
                            if (p == -1):
                                p = 50
                            elif (p == 0):
                                p = 100
                            else:
                                p = 0
                        pic_x.append(p)
                        cnt += 1
                    self.pic_mat.append(pic_x)
                ####################################################################################################
                ## Calling the mapProcessing function to find and return the frontier points,
                # then converting the frontier points locations from map cells into map positions
                frontier_pnts = self.mapProcessing(self.pic_mat)  # Frontier points in map cells

                ## Evaluating the frontier points in meters with respect to the map origin
                frontier_x = []
                frontier_y = []
                for element in frontier_pnts:
                    element = list(element)
                    column_index = element[0]
                    row_index = element[1]
                    x = map_resolution * column_index + ox
                    y = map_resolution * row_index + oy
                    frontier_x.append(x)
                    frontier_y.append(y)

                # The x and y coordinates of the found frontiers (in meter) relative to the map
                self.frontiers.frontier_x = frontier_x
                self.frontiers.frontier_y = frontier_y
                self.frontiers_pub.publish(self.frontiers)

                if continous_running:
                    continue
                else:
                    break
            else:
                rospy.loginfo('Waiting for new maps and maps updates...')
                time.sleep(1)
                continue

    def mapProcessing(self, map_pic):
        ## Evaluating all potential frontier points. The map cells of the detected edges are checked as following: If
        # the cell is a free space (white) and it has an unknown space (grey) next to it, then it is a potential
        # frontier. Also, if the cell is an unknown space (grey) and it has a free space (white) next to it then it
        # is a potential frontier.

        frontier_pnts = []

        # Vectors to be used in moving over the neighborhood cells of a given cell
        dx = [0, -1, -1, -1, 0, 1, 1, 1]
        dy = [1, 1, 0, -1, -1, -1, 0, 1]

        # Performing edge detection on the map_pic picture
        frontier_mat = np.array(map_pic).astype(np.uint8)
        frontier_mat = cv2.Canny(frontier_mat, 100, 200)  # Canny edge detection method

        # Returning the locations of the elements whose values are equal to 255 in matrix 'frontier_mat'
        free_pnts = np.asarray(np.where(frontier_mat == 255)).T.tolist()

        # Checking the detected edges for a valid frontier point
        frontier_mat = np.zeros(np.shape(map_pic), dtype=np.uint8).tolist()
        row, col = np.shape(map_pic)
        for j in range(len(free_pnts)):
            r, c = free_pnts[j]
            if map_pic[r][c] == 100:  # 100 means white color (free space) in the pic "map_pic"
                for i in range(8):  # Moving through the neighborhood cells
                    r1 = r + dx[i]
                    c1 = c + dy[i]

                    if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                        if map_pic[r1][c1] == 50:  # 50 means grey color (unknown space) in the pic "map_pic"
                            frontier_mat[r][c] = 255
                            break
            elif map_pic[r][c] == 50:  # 50 means grey color (unknown space) in the pic "map_pic"
                for i in range(8):
                    r1 = r + dx[i]
                    c1 = c + dy[i]

                    if r1 >= 0 and c1 >= 0 and r1 < row and c1 < col:
                        if map_pic[r1][c1] == 100:  # 100 means white color (free space) in the pic "map_pic"
                            frontier_mat[r][c] = 255
                            break

        # Performing dilation morphological transformation
        frontmat = np.array(frontier_mat).astype(np.uint8)
        kernel = np.ones((5, 5), np.uint8) * 255
        frontmat = cv2.dilate(frontmat, kernel, iterations=3)

        # Making contours from the frontier points found
        contour = cv2.findContours(frontmat.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # Enclosing and filtering the frontier contours
        map_resolution = self.grid.info.resolution
        min_frontier_size_cells = min_frontier_size / map_resolution
        radii = []
        if len(contour) > 0:
            output = np.array(map_pic).astype(np.uint8)
            for i in range(len(contour)):
                c = contour[i]
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                if radius > min_frontier_size_cells:
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    frontier_pnts.append(center)
                    radii.append(radius)
                    output = cv2.circle(output, (int(center[0]), int(center[1])), int(radius), 75, 2)

            if show_frontiers:
                plt.imshow(output, cmap='gray', origin='lower')
                plt.draw()
                plt.pause(0.01)

        self.frontiers.frontier_radius = [r * map_resolution for r in radii]  # Frontiers radii in meter
        return frontier_pnts  # Frontiers points (centers) in map cells

    def isaccessible(self, robot_pos, frontier_pos, free_space_threshold=0, GBFS_mode=1):
        while not self.ctrl_c:
            if self.flagg == 1 and self.init == 1:
                self.flagg = 0
                robot_pos_x = robot_pos[0]  # robot x position in meter
                robot_pos_y = robot_pos[1]  # robot y position in meter
                frontier_pos_x = frontier_pos[0]  # frontier x position in meter
                frontier_pos_y = frontier_pos[1]  # frontier y position in meter

                width = self.grid.info.width
                height = self.grid.info.height
                map_resolution = self.grid.info.resolution
                ox = self.grid.info.origin.position.x
                oy = self.grid.info.origin.position.y
                origin = [ox, oy]
                pnts = self.grid.data  # Map data
                gpnts = np.array(self.newcost).flatten().tolist()  # Global costmap (or global costmap update) data

                # Converting the robot and frontier positions from x-y coordinates into map cell indices
                start_row_index = int(round((robot_pos_y - oy) / map_resolution))
                start_column_index = int(round((robot_pos_x - ox) / map_resolution))
                start_flat_index = width * start_row_index + start_column_index

                goal_row_index = int(round((frontier_pos_y - oy) / map_resolution))
                goal_column_index = int(round((frontier_pos_x - ox) / map_resolution))
                goal_flat_index = width * goal_row_index + goal_column_index

                ## Converting the map data into grayscale picture, where white color represents free space, gray color
                # represents unknown space, and black color represents occupied space
                # In map data: 0 -> free space, -1 -> unknown space, and 100 -> occupied space
                # In grayscal pic: gmin -> black, 50 -> gray, and 100 -> white
                gmin = free_space_threshold
                cnt = 0
                pic_mat_access = []
                for x in range(height):
                    pic_x = []
                    for y in range(width):
                        p = pnts[cnt]
                        g = gpnts[cnt]

                        if (g > gmin):
                            p = 0
                        else:
                            if (p == -1):
                                p = 50
                            elif (p == 0):
                                p = 100
                            else:
                                p = 0
                        pic_x.append(p)
                        cnt += 1
                    pic_mat_access.append(pic_x)

                gpnts = np.array(self.newcost).flatten().tolist()
                start_cell_cost = gpnts[start_flat_index]

                custom_costmap = flatten(pic_mat_access)
                if custom_costmap[start_flat_index] == 0:  # The robot is in very proximity of an obstacle
                    rospy.loginfo("The cost of the map cell at the robot's location= " + str(start_cell_cost))
                    isaccessible = -1  # The frontier point accessibility cannot be checked by GBFS
                    return [isaccessible, start_cell_cost]


                path = self.GBFS_obj.greedy_bfs(start_flat_index, goal_flat_index, width, height, custom_costmap, map_resolution,
                                           origin, GBFS_mode)

                if path:
                    if show_frontiers == True:
                        pic = np.array(pic_mat_access).astype(np.uint8)
                        pic = cv2.circle(pic, (start_column_index, start_row_index), 20, 75, 2)
                        pic = cv2.circle(pic, (goal_column_index, goal_row_index), 20, 75, 2)

                        for flatmap_index in path:
                            column_index = flatmap_index % width
                            row_index = flatmap_index // width
                            pic = cv2.circle(pic, (column_index, row_index), 5, 75, 2)

                        # plt.imshow(pic, cmap='gray', origin='lower')
                        # plt.draw()
                        # plt.pause(0.01)
                    isaccessible = 1  # The frontier point is accessible
                    return [isaccessible, start_cell_cost]

                else:
                    isaccessible = 0  # The frontier point is not accessible
                    return [isaccessible, start_cell_cost]

    def shutdownhook(self):
        self.ctrl_c = True

def flatten(t):
    return [item for sublist in t for item in sublist]

def main():
    rospy.init_node('find_frontier')  # make node
    frontiers = find_frontiers()
    rospy.sleep(1)
    frontiers.findFrontier(True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
