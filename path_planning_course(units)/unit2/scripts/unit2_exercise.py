#! /usr/bin/env python

"""
Dijkstra's algorithm path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit2 unit2_exercise.launch
"""

import rospy

def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 8 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 1

  upper = index - width
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = index - 1
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
      neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width - 1):
    if costmap[upper_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
      neighbors.append([upper_right, step_cost])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower_left = index + width - 1
  if lower_left < height * width and lower_left % width != 0:
    if costmap[lower_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
      neighbors.append([lower_left, step_cost])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width - 1):
    if costmap[lower_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
      neighbors.append([lower_right, step_cost])

  return neighbors


def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz = None):
    ''' 
    Performs Dijkstra's shortest path algorithm search on a costmap with a given start and goal node
    '''
    # Initialize open list with the start node and its g_cost
    open_list = []
    open_list.append([start_index, 0])
    
    # Initialize closed list to keep track of visited nodes
    closed_list = set()
    
    # Initialize parents dictionary to map child nodes to their parents
    parents = {}
    
    # Initialize g_costs dictionary to keep track of g_cost values for each node
    g_costs = {start_index: 0}
    
    # Initialize shortest path list to store the final path
    shortest_path = []
    
    # Initialize path found flag to False
    path_found = False
    
    # Log a message indicating that initialization is done
    rospy.loginfo('Dijkstra: Initialization done!')
    
    # Main loop over open_list
    while open_list:
        # Sort open_list by g_cost in ascending order
        open_list.sort(key=lambda x: x[1])
        
        # Pop the node with the lowest g_cost and set it as current_node
        current_node = open_list.pop(0)[0]
        
        # Add current_node to closed_list
        closed_list.add(current_node)
        
        # Optional: visualize the current node
        if grid_viz:
            grid_viz.set_color(current_node, "pale yellow")
        
        # Check if goal has been reached
        if current_node == goal_index:
            path_found = True
            break
        
        # Find neighboring nodes
        neighbours = find_neighbors(current_node, width, height, costmap, resolution)
        
        # Loop over all neighbours
        for neighbor in neighbours:
            # Unpack neighbor into index and step_cost
            neighbor_index, step_cost = neighbor
            
            # Skip if neighbor is already in closed_list
            if neighbor_index in closed_list:
                continue
            
            # Calculate the travel cost to the neighbor
            g_cost = g_costs[current_node] + step_cost
            
            # Check if neighbor is already in open_list
            found_in_open_list = False
            for i, (node_index, node_cost) in enumerate(open_list):
                if node_index == neighbor_index:
                    found_in_open_list = True
                    open_list_index = i
                    break
            
            if found_in_open_list:
                # Case 1: Neighbor is already in open_list
                if g_cost < g_costs[neighbor_index]:
                    # Update g_cost and parent
                    g_costs[neighbor_index] = g_cost
                    parents[neighbor_index] = current_node
                    open_list[open_list_index][1] = g_cost
            else:
                # Case 2: Neighbor is not in open_list
                g_costs[neighbor_index] = g_cost
                parents[neighbor_index] = current_node
                open_list.append([neighbor_index, g_cost])
                
                # Optional visualization
                if grid_viz:
                    grid_viz.set_color(neighbor_index, 'orange')
    
    # Optional log message for when traversal is complete
    rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

    # Check if the goal node was found
    if not path_found:
        rospy.logwarn('Dijkstra: No path found!')
        return shortest_path  # Return an empty list as no path was found

    # Reconstruct the shortest path from goal to start
    node = goal_index
    while node != start_index:
        shortest_path.append(node)
        node = parents[node]
    
    # Add the start node to the path
    shortest_path.append(start_index)

    # Reverse the path to start from start_index and end at goal_index
    shortest_path.reverse()

    # Log that the path reconstruction is complete
    rospy.loginfo('Dijkstra: Done reconstructing path')

    return shortest_path


