#!/usr/bin/env python

import math
from typing import Dict, List, Tuple
from queue import PriorityQueue
import random
import time

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, ColorRGBA
from LOS import LOS


class Visualisation:
    def __init__(self):
        rospy.init_node('rviz_marker')
        self.marker_pub = marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        self.marker_lifetime = rospy.Duration(0.0)
        self.path_marker = self.init_path_marker()
        self.obstacle_marker = self.init_obstacle_marker(1)
        self.alpha = 1.0

    def init_path_marker(self):
        # Path Marker (Line List)
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.ns = "Path" # unique ID
        path_marker.action = Marker().ADD
        path_marker.type = Marker().LINE_LIST
        path_marker.lifetime = self.marker_lifetime
        path_marker.pose.position.x = 0.0
        path_marker.pose.position.y = 0.0
        path_marker.pose.position.z = 0.0
        path_marker.pose.orientation.x = 0.0
        path_marker.pose.orientation.y = 0.0
        path_marker.pose.orientation.z = 0.0
        path_marker.pose.orientation.w = 1.0
        
        path_marker.scale.x = 0.1
        return path_marker

    def init_obstacle_marker(self, idx):
        rectangle_marker = Marker()
        rectangle_marker.header.frame_id = "map"
        rectangle_marker.ns = "Rectangle" + str(idx) # unique ID
        rectangle_marker.action = Marker().ADD
        rectangle_marker.type = Marker().CUBE
        rectangle_marker.lifetime = self.marker_lifetime

        return rectangle_marker

    def publish_path(self, path):
        self.path_marker.points.clear()
        self.path_marker.colors.clear()
        self.path_marker.header.stamp = rospy.Time.now()
        
        color = ColorRGBA()
        color.a = self.alpha
        color.r = 0.1
        color.g = 0.8
        color.b = 0.1
        for segment in path:
            start_pt , end_pt = segment
            self.path_marker.points.append(Point(start_pt[0], start_pt[1], 0))
            self.path_marker.colors.append(color)
            
            self.path_marker.points.append(Point(end_pt[0], end_pt[1], 0))
            self.path_marker.colors.append(color)

        return self.path_marker

    def publish_obstacle(self, obstacle, obstacle_marker, idx):
        # self.obstacle_marker.points.clear()
        # self.obstacle_marker.colors.clear()
        obstacle_marker.header.stamp = rospy.Time.now()
        
        color = ColorRGBA()
        color.a = self.alpha
        color.r = 0.8
        color.g = 0.1
        color.b = 0.1

        self.obstacle_marker.id = idx
        bottom_left, top_right = obstacle

        scale = Vector3(top_right[0] - bottom_left[0] + 1, top_right[1] - bottom_left[1] +1, 1)
        # scale = Vector3(3,2 , 1)
        
        obstacle_marker.scale = scale

        mid_point_x = bottom_left[0] + (top_right[0] - bottom_left[0]) / 2
        mid_point_y = bottom_left[1] + (top_right[1] - bottom_left[1]) / 2 
        # print(mid_point_x, mid_point_y)
        # print(top_right[0] - bottom_left[0] + 1, top_right[1] - bottom_left[1] +1)
        
        obstacle_pose = Pose()
        obstacle_pose.position.x = mid_point_x
        obstacle_pose.position.y = mid_point_y
        obstacle_pose.position.z = 0
        obstacle_marker.pose = obstacle_pose
        #TODO change points
        # obstacle_marker.points.append(Point(mid_point_x ,mid_point_y,0))
        
        # obstacle_marker.colors.append(color)
        obstacle_marker.color = color

        return obstacle_marker


class Robot:
    def __init__(self, startingPos):
        self.startingPos = startingPos
        self.currPos = self.startingPos
        self.movements = [(1, 0), (1, 1), (0, 1), (0, 0)]

    def getValidActions(self, maxRowIndex, maxColIndex, obstaclesPositions):
        col, row = self.currPos
        validActions = []
        for movement in self.movements:
            newPos = (movement[0] + col, movement[1] + row)
            if self.validSquareToMoveTo(newPos, maxRowIndex, maxColIndex, obstaclesPositions):
                validActions.append(newPos)

        return validActions

    def validSquareToMoveTo(self, pos, maxRowIndex, maxColIndex, obstaclesPositions):
        withinGrid = pos[0] >= 0 and pos[1] >= 0 and \
            pos[0] <= maxColIndex and pos[1] <= maxRowIndex
        freeSpace = self.isFreeSpace(pos, obstaclesPositions) #pos not in obstaclesPositions

        return withinGrid and freeSpace

    def isFreeSpace(self, pos, obstaclesPositions):
        posCol, posRow = pos
        # print(pos, obstaclesPositions)
        for p1, p2 in obstaclesPositions:
            if not p1 and not p2:
                return True
            if posCol >= p1[0] and posCol <= p2[0] and \
                posRow >= p1[1] and posRow <= p2[1]:
                return False

        return True 


    def updatePos(self, newPos):
        self.currPos = newPos

class Grid:
    def __init__(self, numOfRows, numOfCols, obstacles, goalPos, inflation_radius = 1):
        self.numOfRows = numOfRows
        self.numOfCols = numOfCols
        self.obstacles = obstacles #[ [(topleftcol, topleftrow), (bottomrightcol, bottomrightrow)] ... ]
        # self.obstacles = list(map(lambda x: [(x[0][0] - inflation_radius, x[0][1] - inflation_radius), (x[1][0] + inflation_radius, x[1][1] + inflation_radius)], self.obstacles))
        self.goalPos = goalPos

class Node:
    def __init__(self, state, parent=None, action=None, pathCost=0, goalCost=0) -> None:
        self.state = state
        self.parent = parent
        self.action = action
        self.pathCost = pathCost
        self.goalCost = goalCost

    def getTotalCost(self):
        return self.pathCost + self.goalCost

    def __lt__(self, other):
        return self.getTotalCost() < other.getTotalCost()

class Maze:
    def __init__(self, grid: Grid, robot: Robot) -> None:
        self.grid = grid
        self.robot = robot
        self.defaultStepCost = 1

    def getInitialState(self):
        return self.robot.startingPos
    
    def isGoal(self, state):
        return state == self.grid.goalPos
        # return state.piece.currPos in self.board.goalPos

    def getActions(self, state):
        self.robot.updatePos(state)
        maxRowIndex = self.grid.numOfRows - 1
        maxColIndex = self.grid.numOfCols - 1
        obstaclePositions = self.grid.obstacles
        return self.robot.getValidActions(maxRowIndex, maxColIndex, obstaclePositions)

    # transition function
    def result(self, state, action):
        assert len(action) == 2
        # state.piece.updatePos((action[0], action[1]))
        self.robot.updatePos((action[0], action[1]))
        return self.robot.currPos

    def getActionCost(self, currState, finalState):
        return euclideanDistance(currState, finalState)
        # return self.defaultStepCost

    def getGoalCost(self, currState):
        #diagonal heuristic
        # colDiff, rowDiff = abs(self.grid.goalPos[0] - currState[0]), abs(self.grid.goalPos[1] - currState[1])
        # value = max(colDiff, rowDiff)

        # #euclidean heuristic 
        # value = euclideanDistance(currState, self.grid.goalPos)

        # #manhattan dist heuristic 
        colDiff, rowDiff = abs(self.grid.goalPos[0] - currState[0]), abs(self.grid.goalPos[1] - currState[1])
        value = colDiff + rowDiff
        # value = 0 # use this value if running Dijkstra
        return value

def euclideanDistance(p1, p2):
    dist = (p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1])
    return math.sqrt(dist)
        
def expand(maze:Maze, node: Node) -> List[Node]:
    state = node.state
    actions = maze.getActions(state)
    childNodes = []
    for action in actions:
        newState = maze.result(state, action)
        cost = node.pathCost + maze.getActionCost(state, newState)
        goalCost = maze.getGoalCost(newState)
        childNode = Node(newState, parent=node, action=action, pathCost=cost, goalCost=goalCost)
        childNodes.append(childNode)

    return childNodes


def search(maze: Maze):
    initialState = maze.getInitialState()
    node = Node(initialState)
    # print(initialState)
    frontier = PriorityQueue()
    frontier.put(node)
    reached = {initialState: node}
    
    while not frontier.empty():
        node = frontier.get()
        if maze.isGoal(node.state):
            return listOfValidMoves(node), len(reached), node.pathCost
        childNodes = expand(maze, node)
        for child in childNodes:
            state = child.state
            
            # BASIC THETA*
            # if node.parent is not None and is_los(node.parent.state, state):
            #     # print("went here")
            #     cost = node.parent.pathCost + euclideanDistance(node.parent.state, state)
            #     if state not in reached or (cost < reached[state].pathCost):
            #         child.pathCost = cost
            #         child.parent = node.parent
            #         reached[state] = child
            #         frontier.put(child)

            # else:
            #     if state not in reached or (child.pathCost < reached[state].pathCost):
            #         # print("went there")
            #         reached[state] = child
            #         frontier.put(child)
            # BASIC THETA*

            if state not in reached or (child.pathCost < reached[state].pathCost):
                reached[state] = child
                frontier.put(child)
    
    return listOfValidMoves(None), len(reached), 0

def listOfValidMoves(node: Node):
    if node == None:
        return []
    col, row = node.state
    move = (col, row)
    sequence = [move]
    while node.parent != None:
        col, row = node.parent.state
        move = (col, row)
        sequence.append(move)
        node = node.parent
    sequence.reverse()
    idx = 0
    moves = []
    while idx < len(sequence) - 1:
        moves.append([sequence[idx], sequence[idx + 1]])
        idx += 1
    return moves

def run_planner(obstacles, goal):
    grid = Grid(100, 100, obstacles, goal) #[[(4,4), (6,6)]]
    maze = Maze(grid, Robot((0,0)))
    moves, nodesExplored, pathCost = search(maze)
    return moves, nodesExplored

def post_process(path, obstacles):
    initial_path = [path[0][0]]
    initial_path.extend(list(map(lambda x: x[1], path)))
    turning_points = [initial_path[0]]

    #get turning points
    for i in range(2, len(initial_path)):
        pos_next = initial_path[i]
        pos_curr = initial_path[i-1]
        pos_prev = initial_path[i-2]

        Dx_next = pos_next[0] - pos_curr[0]
        Dy_next = pos_next[1] - pos_curr[1]
        Dx_prev = pos_curr[0] - pos_prev[0]
        Dy_prev = pos_curr[1] - pos_prev[1]

        if (abs(Dx_next * Dy_prev - Dy_next * Dx_prev)) > 1e-5:
            turning_points.append(pos_curr)
    
    if initial_path[-1] not in turning_points:
        turning_points.append(initial_path[-1])


    # any angle path
    los = LOS()
    post_processed_path = []    
    k = 0
    post_processed_path.append(turning_points[0]) 
    for i in range(1, len(turning_points)-1):
        start_pt = post_processed_path[k]
        end_pt = turning_points[i+1]
        sight = los.get(start_pt, end_pt)
        for pt in sight:
            if not isFreeSpace(pt, obstacles):
                k+=1 
                post_processed_path.append(turning_points[i])

    post_processed_path.append(turning_points[-1])
    ##TODO remove duplicates
    return format_path(post_processed_path)

def format_path(path):
    #remove duplicates
    path = list(dict.fromkeys(path)) 
    idx = 0
    moves = []
    while idx < len(path) - 1:
        moves.append([path[idx], path[idx + 1]])
        idx += 1
    return moves

def isFreeSpace(pos, obstaclesPositions):
    posCol, posRow = pos
    # print(pos, obstaclesPositions)
    for p1, p2 in obstaclesPositions:
        if not p1 and not p2:
            return True
        if posCol >= p1[0] and posCol <= p2[0] and \
            posRow >= p1[1] and posRow <= p2[1]:
            return False

    return True 

def is_los(src, tgt):
    los = LOS()
    sight = los.get(src, tgt)
    for pt in sight:
        if not isFreeSpace(pt, inflated_obstacles):
            return False
    return True

def generate_random_goals(no_of_goals, max_row_index, max_col_index, obstaclePositions):
    random_goals = []
    while len(random_goals) < no_of_goals:
        random_x, random_y = random.randint(0, max_col_index), random.randint(0, max_row_index)
        if (random_x, random_y) in random_goals or not isFreeSpace((random_x, random_y), obstaclePositions):
            continue
        random_goals.append((random_x, random_y))
    return random_goals

def calculate_distance_travelled(path):
    total_distance = 0
    for segment in path:
        total_distance += euclideanDistance(segment[0], segment[1])
    return total_distance


obstacles = [[(4,4), (6,6)], [(2,2), (6,3)], [(20,30), (30,45)], [(30,70), (70,90)], [(45,20), (55,40)], [(45,50), (55,70)], [(10,20), (40,25)]]
inflation_radius = 0
inflated_obstacles = list(map(lambda x: [(x[0][0] - inflation_radius, x[0][1] - inflation_radius), (x[1][0] + inflation_radius, x[1][1] + inflation_radius)], obstacles))
#random goals listed below is generated using the generate_random_goal function
random_goals = [(85, 2), (93, 22), (24, 87), (72, 83), (79, 86), (99, 89), (58, 2), (59, 39), (40, 7), (56, 4), (1, 52), (55, 13), (44, 14), (15, 78), (39, 18), (90, 34), (70, 14), (23, 72), (13, 76), (4, 35), (74, 39), (48, 11), (53, 9), (97, 49), (92, 20), (79, 63), (43, 31), (62, 51), (22, 81), (73, 54), (51, 47), (72, 20), (73, 90), (8, 36), (83, 65), (87, 4), (14, 7), (96, 1), (17, 28), (74, 30), (20, 17), (64, 66), (87, 19), (84, 75), (58, 52), (26, 10), (79, 88), (15, 4), (65, 67), (58, 11)]


if __name__ == "__main__":
    ############# generating random goal points and collecting performance data #######
    # num_of_random_goals = 50
    # random_goals = generate_random_goals(num_of_random_goals, 99, 99, inflated_obstacles)
    # print(random_goals)
    # total_time = total_distance = total_nodes = 0
    # for goal in random_goals:
    #     begin = time.time()
    #     path, nodesExplored = run_planner(inflated_obstacles, goal)  
    #     # path = post_process(path, inflated_obstacles)
    #     end = time.time()
    #     total_time = total_time + (end - begin)   
    #     total_nodes += nodesExplored
    #     total_distance += calculate_distance_travelled(path)
    # print(total_time/num_of_random_goals, total_nodes/num_of_random_goals,total_distance/num_of_random_goals)
    
    path, nodesExplored = run_planner(inflated_obstacles, (99,99))
    # path = post_process(path, inflated_obstacles) ##uncommed this for A* Post Smooting
    # print(post_process(path, inflated_obstacles))
    # print(nodesExplored)
# 
    ###################### Visualisation of path and obstacles ###########################################
    viz = Visualisation()
    path_marker = viz.publish_path(path)
    

    obstacle_markers = []
    count = 1
    for obstacle in obstacles:
        obstacle_marker = viz.publish_obstacle(obstacle, viz.init_obstacle_marker(count), count)
        obstacle_markers.append(obstacle_marker)
        count +=1 


    while not rospy.is_shutdown():
        for obstacle_marker in obstacle_markers:
            viz.marker_pub.publish(obstacle_marker)
        viz.marker_pub.publish(path_marker)
        rospy.rostime.wallsleep(1.0)