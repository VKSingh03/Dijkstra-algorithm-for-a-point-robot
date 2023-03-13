# Project by Vineet Singh 

# GITHUB Link to code:         https://github.com/VKSingh03/Dijkstra-algorithm-for-a-point-robot

import numpy as np
import argparse
import math
import queue
import cv2 
import timeit

# Node class 
class Node:
    def __init__(self, state, parent, move, cost): 
        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost

    def get_state(self):
        return self.state
    def get_parent(self):
        return self.parent
    def get_move(self):
        return self.move
    def get_cost(self):
        return self.cost
    def get_parent_state(self):
        if self.get_parent() is None:
            return None
        return self.get_parent().get_state()
     
    def getFullPath(self):
        # moves = []
        nodes = []
        current_node = self
        while(current_node.get_move() is not None):
            # moves.append(current_node.get_move())
            nodes.append(current_node)
            current_node = current_node.get_parent()
        nodes.append(current_node)
        # moves.reverse()
        nodes.reverse()
        return nodes
    
    def __lt__(self,other):
        return self.get_state() < other.get_state()
    
def getBranches(node):
    branches = []
    branches.append(Node(ActionMoveUp(node.get_state()), node, 'U', node.get_cost()+1.0))
    branches.append(Node(ActionMoveUpRight(node.get_state()) ,node, 'UR', node.get_cost()+1.4))
    branches.append(Node(ActionMoveRight(node.get_state()), node, 'R', node.get_cost()+1.0))
    branches.append(Node(ActionMoveDownRight(node.get_state()) ,node, 'DR', node.get_cost()+1.4))
    branches.append(Node(ActionMoveDown(node.get_state()), node, 'D', node.get_cost()+1.0))
    branches.append(Node(ActionMoveDownLeft(node.get_state()) ,node, 'DL', node.get_cost()+1.4))
    branches.append(Node(ActionMoveLeft(node.get_state()), node, 'L', node.get_cost()+1.0))
    branches.append(Node(ActionMoveUpLeft(node.get_state()) ,node, 'UL', node.get_cost()+1.4))
    b = [branch for branch in branches if branch.get_state() is not None]
    return b

def ActionMoveUp(state):
    state_copy = state.copy()
    state_copy[1] = state_copy[1]+1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy
        
def ActionMoveUpRight(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]+1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

def ActionMoveRight(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

def ActionMoveDownRight(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]-1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

def ActionMoveDown(state):
    state_copy = state.copy()
    state_copy[1] = state_copy[1]-1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy
    
def ActionMoveDownLeft(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]-1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

def ActionMoveLeft(state):
    state_copy =state.copy()
    state_copy[0] = state_copy[0]-1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

def ActionMoveUpLeft(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]+1
    if (ObstacleSpace(state_copy)) :
        return None
    else:
        return state_copy

# Function to add obstacle space to canvas
def ObstacleSpacetoMap(canvas):
    # In cartesian
    for i in range(0,599,1):
        for j in range(0,249,1):
            # Adding upper rectangle
            if (95<i<155 and 0<j<105):
                canvas[j,i] = [255,255,1]

            # Adding lower rectangle
            elif (95<i<155 and 155<j<250):
                canvas[j,i] = [255,255,1]
                
            # Adding Hexagon 
            elif (((70*j-42*i-1750)<=0)& ((70*j+42*i-26950)<=0) & (i<=370) & ((70*j-42*i+9450)>=0)& ((70*j+42*i-15750)>=0) &(i>=230)):
                canvas[j,i] = [255,255,1]
                
            # Adding Out of bounds
            elif ((i<=5)or (i>=595) or (j<=5) or(j>=245)):
                canvas[j,i] = [255,255,1]
                
            # Adding triangle
            elif((i>=455) & ((60*j+105*i-61575)<=0) & ((60*j-105*i+46575)>=0)):
                canvas[j,i] = [255,255,1]
                
            else :
                pass # allowed_states.append([i, j])
    return canvas

# Function to check obstacle space for every state
def ObstacleSpace(point):
            i, j = point
            # In cartesian
            # Adding upper rectangle
            if (95<i<155 and 0<j<105):
                # print("Approaching upper rectangle")
                return 1

            # Adding lower rectangle
            elif (95<i<155 and 155<j<250):
                # print("Approaching lower rectangle")
                return 1
                
            # Adding Hexagon 
            elif (((70*j-42*i-1750)<=0)& ((70*j+42*i-26950)<=0) & (i<=370) & ((70*j-42*i+9450)>=0)& ((70*j+42*i-15750)>=0) &(i>=230)):
                # print("Approaching hexagon")
                return 1
                
            # Adding Out of bounds
            elif ((i<=5)or (i>=595) or (j<5) or(j>245)):
                # obstacle_space.append([i,j])
                # print("Approaching boundary")
                return 1
            
            # Adding triangle
            elif((i>=455) & ((60*j+105*i-61575)<=0) & ((60*j-105*i+46575)>=0)):
                # print("Approaching traingle")
                return 1

            else :
                return 0

# Function to change from Cartesian to OpenCv scale
def updateMapViz(canvas, state, color):
    X, Y, _ = canvas.shape
    transformed_y = state[1] 
    transformed_x = state[0] 
    # cv2.flip(canvas,0)
    canvas[transformed_y,transformed_x] = color
    return canvas

def dijkstra(start_point, goal_point,result):
    # Creating Priority queue
    nodes = queue.PriorityQueue()
    #Adding first node in priority queue
    init_node = Node(start_point, None, None, 0)
    nodes.put((init_node.get_cost(), init_node))
    # Creating canvas for final print
    canvas_size = [250,600]
    canvas = np.zeros([canvas_size[0], canvas_size[1], 3], dtype=np.uint8)
    canvas = ObstacleSpacetoMap(canvas)
    canvas = updateMapViz(canvas, start_point, [0,255,255])
    canvas = updateMapViz(canvas, goal_point, [0,255,255])

    goal_reached = False
    # Creating node array of size equal to given canvas
    node_array = np.array([[Node([i,j], None, None, math.inf) for j in range(250)] for i in range(600)])
    closed_list = []
    print("Starting Algorithm.......")
    while (not nodes.empty()):
        
        current_node = nodes.get()[1]
        closed_list.append(current_node.get_state())
        canvas = updateMapViz(canvas, current_node.get_state(), [0, 255, 255])
        result.write(cv2.flip(canvas,0))

        if (current_node.get_state() == goal_point):
            print('Goal reached')
            print("The cost of path: ", current_node.get_cost())
            node_path = current_node.getFullPath()
            goal_reached = True
            for node in node_path:
                pos = node.get_state()
                canvas = updateMapViz(canvas, pos, [0, 0, 0])
                result.write(cv2.flip(canvas,0))
            for i in range(900):
                result.write(cv2.flip(canvas,0))
        else:
            # Creating child states for the current node
            moves = getBranches(current_node)
            for move in moves:
                # Checking if new state is in closed list 
                if (move.get_state() in closed_list):
                    continue
                child_state = move.get_state()
                new_cost = move.get_cost()
                # Checking if the new state is explored or not
                if (node_array[child_state[0], child_state[1]].get_cost()==math.inf):
                    node_array[child_state[0], child_state[1]]=move
                    nodes.put((new_cost,move))
                # Checking if the ew state 
                else: 
                    if (new_cost < node_array[child_state[0], child_state[1]].get_cost()):
                        node_array[child_state[0], child_state[1]] = move
                        nodes.put((new_cost, move))
        if (goal_reached): 
            break
       

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--InitState",nargs='+', type=int, help = 'Initial state for the matrix')
    parser.add_argument("--GoalState",nargs='+', type=int, help = 'Goal state for the matrix')
    parser.add_argument('--SaveFolderName', default='./', help='Path to store generated files')

    Args = parser.parse_args()
    result_folder = Args.SaveFolderName
    initial_point = Args.InitState
    goal_point = Args.GoalState
    initial_point = list(initial_point)
    goal_point = list(goal_point)

    if (ObstacleSpace(initial_point)):
        print( "Start point in obstacle space or out of bound")
        exit(0)

    if (ObstacleSpace(goal_point)):
        print( "Goal point in obstacle space or out of bound")
        exit(0)

    file_name = result_folder + "dijkstra_simulation.avi"
    space_size = [250, 600]
    size_y, size_x = space_size
    result = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*'MJPG'), 800, (size_x, size_y))

    print("Initial state is: ", initial_point)
    print("Goal state is: ", goal_point)

    start = timeit.default_timer()
    dijkstra(initial_point, goal_point, result)
    stop = timeit.default_timer()
    print('Time taken by Algorithm: ', stop-start)
    print("storing it to file ...")

if __name__ == "__main__":
    main()