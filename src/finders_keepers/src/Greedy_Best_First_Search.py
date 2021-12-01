#! /usr/bin/env python
"""
File         : Greedy_Best_First_Search.py
Information  : This code file provides reedy Best-First Search path planning algorithm
Author       : Mohamed Tolba
Last Modified: 16 Nov 2021
Credit       : This algorithm is result of my understanding to the algorithm built by Roberto Zegers R.
               (https://www.theconstructsim.com/)
"""

import math

class GBFS():
    def __init__(self):
        self.move_step = 5 * 0 + 10  # Step size of movement in checking a map index neighbours
        self.tol = 0.25 # The maximum acceptable distance between the frontier and last accessible map cell
        # rospy.loginfo("GBFS.....")

    def find_neighbors(self, flatmap_index, width, height, costmap, orthogonal_step_cost=1, mode=1):
        # Identifies neighbor nodes inspecting the 8 adjacent neighbors
        # Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
        # Returns a list with valid neighbour nodes as [index, step_cost] pairs
        # 'costmap' is a 1D array grayscale picture where:
        # 0 value -> black color (obstacle)
        # 50 value -> grey color (unknown space)
        # 100 value -> white color (free space)

        neighbors = []
        # length of diagonal = length of one side by the square root of 2 (1.41421)
        diagonal_step_cost = orthogonal_step_cost * 1.41421

        # threshold values used to reject neighbor nodes
        obstacle_cost = 0
        unknown_space_cost = 50
        free_space_cost = 100
        if mode == 1:
            selected_cost = unknown_space_cost  # The neighbours with cost greater than the unknown_space_cost will
            # be accepted
        else:
            selected_cost = obstacle_cost  # The neighbours with cost greater than the obstacle_cost will be accepted

        # Evaluate the column_index and the row_index corresponding to the flatmap_index
        # to make the moving process to the neighbourhood nodes easier and more manageable
        # as if the input costmap is a 2D array
        column_index = flatmap_index % width
        row_index = flatmap_index // width

        # Upper neighbor
        upper_row_index = row_index + self.move_step
        if upper_row_index < height:
            neighbor_flatmap_index = width * upper_row_index + column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = orthogonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Lower neighbor
        lower_row_index = row_index - self.move_step
        if lower_row_index >= 0:
            neighbor_flatmap_index = width * lower_row_index + column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = orthogonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Right neighbor
        right_column_index = column_index + self.move_step
        if right_column_index < width:
            neighbor_flatmap_index = width * row_index + right_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = orthogonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Left neighbor
        left_column_index = column_index - self.move_step
        if left_column_index >= 0:
            neighbor_flatmap_index = width * row_index + left_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = orthogonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Upper-Right neighbor
        upper_right_column_index = column_index + self.move_step
        upper_right_row_index = row_index + self.move_step
        if upper_right_column_index < width and upper_right_row_index < height:
            neighbor_flatmap_index = width * upper_right_row_index + upper_right_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = diagonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Upper-Left neighbor
        upper_left_column_index = column_index - self.move_step
        upper_left_row_index = row_index + self.move_step
        if upper_left_column_index >= 0 and upper_left_row_index < height:
            neighbor_flatmap_index = width * upper_left_row_index + upper_left_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = diagonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Lower-Right neighbor
        lower_right_column_index = column_index + self.move_step
        lower_right_row_index = row_index - self.move_step
        if lower_right_column_index < width and lower_right_row_index >= 0:
            neighbor_flatmap_index = width * lower_right_row_index + lower_right_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = diagonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        # Lower-Left neighbor
        lower_left_column_index = column_index - self.move_step
        lower_left_row_index = row_index - self.move_step
        if lower_left_column_index >= 0 and lower_left_row_index >= 0:
            neighbor_flatmap_index = width * lower_left_row_index + lower_left_column_index
            if costmap[neighbor_flatmap_index] > selected_cost:
                step_cost = diagonal_step_cost
                neighbors.append([neighbor_flatmap_index, step_cost])

        return neighbors

    def indexToWorld(self, flatmap_index, map_width, map_resolution, map_origin=[0, 0]):
        # Converts a flatmap index value to world coordinates (meters)
        # flatmap_index: a linear index value, specifying a cell/pixel in an 1-D array
        # map_width: number of columns in the occupancy grid
        # map_resolution: side length of each grid map cell in meters
        # map_origin: the x,y position in grid cell coordinates of the world's coordinate origin
        # Returns a list containing x,y coordinates in the world frame of reference
        # convert to x,y grid cell/pixel coordinates
        column_index = flatmap_index % map_width
        row_index = flatmap_index // map_width  # '//' operator = Floor division
        # convert to world coordinates
        x = map_resolution * column_index + map_origin[0]
        y = map_resolution * row_index + map_origin[1]

        return [x, y]

    def greedy_bfs(self, start_index, goal_index, width, height, costmap, resolution, origin, mode=1):
        # Performs Greedy Best-First Search on a costmap (1D array) with a given start and goal node

        # create an open_list
        open_list = []

        # set to hold already processed nodes
        closed_list = set()

        # dict for mapping children to parent
        parents = dict()

        # dict for mapping h costs (heuristic costs) to nodes
        h_costs = dict()

        # determine the h cost (heuristic cost) for the start node
        start_node_xy = self.indexToWorld(start_index, width, resolution, origin)
        goal_node_xy = self.indexToWorld(goal_index, width, resolution, origin)
        h_cost = euclidean_distance(start_node_xy, goal_node_xy)

        # set the start's node h_cost
        h_costs[start_index] = h_cost

        # add start node to open list
        open_list.append([start_index, h_cost])

        shortest_path = []

        path_found = False
        # rospy.loginfo('Greedy BFS: Done with initialization')

        # Main loop, executes as long as there are still nodes inside open_list
        while open_list:

            # sort open_list according to the lowest 'h_cost' value (second element of each sublist)
            open_list.sort(key=lambda x: x[1])

            # extract the first element (the one with the lowest 'h_cost' value)
            current_node = open_list.pop(0)[0]

            # print[current_node, h_costs[current_node]]

            # Close current_node to prevent from visiting it again
            closed_list.add(current_node)

            # If current_node is the goal, exit the main loop
            if current_node == goal_index or h_costs[current_node] <= self.tol:
                path_found = True
                break

            # Get neighbors of current_node
            neighbors = self.find_neighbors(current_node, width, height, costmap, resolution, mode)

            # Loop neighbors
            for neighbor_index, step_cost in neighbors:

                # Check if the neighbor has already been visited
                if neighbor_index in closed_list:
                    continue

                # pure heuristic 'h_cost'
                neighbor_xy = self.indexToWorld(neighbor_index, width, resolution, origin)
                h_cost = euclidean_distance(neighbor_xy, goal_node_xy)
                # h_cost = manhattan_distance(from_xy, to_xy)

                # Check if the neighbor is in open_list
                in_open_list = False
                for idx, element in enumerate(open_list):
                    if element[0] == neighbor_index:
                        in_open_list = True
                        break

                # CASE 1: neighbor already in open_list
                if in_open_list:
                    if h_cost < h_costs[neighbor_index]:
                        continue

                # CASE 2: neighbor not in open_list
                else:
                    # Set the node's heuristic cost
                    h_costs[neighbor_index] = h_cost
                    parents[neighbor_index] = current_node
                    # Add neighbor to open_list
                    open_list.append([neighbor_index, h_cost])

        # rospy.loginfo('Greedy BFS: Done traversing nodes in open_list')

        if not path_found:
            # rospy.logwarn('Greedy BFS: No path found!')
            return shortest_path

        # Reconstruct path by working backwards from target
        if path_found:
            # node = goal_index
            node = current_node
            shortest_path.append(goal_index)
            while node != start_index:
                shortest_path.append(node)
                # get next node
                node = parents[node]
        # reverse list
        # shortest_path = shortest_path[::-1]
        shortest_path.reverse()
        # rospy.loginfo('Greedy BFS: Done reconstructing path')

        return shortest_path

def manhattan_distance(pnt1, pnt2):
    x1 = pnt1[0]
    y1 = pnt1[1]
    x2 = pnt2[0]
    y2 = pnt2[1]

    distance = abs(x1 - x2) + abs(y1 - y2)
    return distance

def euclidean_distance(pnt1, pnt2):
    x1 = pnt1[0]
    y1 = pnt1[1]
    x2 = pnt2[0]
    y2 = pnt2[1]
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance
