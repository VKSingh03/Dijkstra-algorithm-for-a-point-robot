import numpy as np
import argparse
import math
import queue
import cv2 
import matplotlib.pyplot as plt
import timeit

canvas_video = []
allowed_states = []
obstacle_space = []
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
        moves = []
        nodes = []
        current_node = self
        while(current_node.get_move() is not None):
            moves.append(current_node.get_move())
            nodes.append(current_node)
            current_node = current_node.get_parent()
        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        return moves, nodes
    
    def __lt__(self,other):
        return self.get_state() < other.get_state()
    
def getBranches(node):
    branches = []
    branches.append(Node(ActionMoveUp(node), node, 'U', node.get_cost()+1.0))
    branches.append(Node(ActionMoveUpRight(node) ,node, 'UR', node.get_cost()+1.4))
    branches.append(Node(ActionMoveRight(node), node, 'R', node.get_cost()+1.0))
    branches.append(Node(ActionMoveDownRight(node) ,node, 'DR', node.get_cost()+1.4))
    branches.append(Node(ActionMoveDown(node), node, 'D', node.get_cost()+1.0))
    branches.append(Node(ActionMoveDownLeft(node) ,node, 'DL', node.get_cost()+1.4))
    branches.append(Node(ActionMoveLeft(node), node, 'L', node.get_cost()+1.0))
    branches.append(Node(ActionMoveUpLeft(node) ,node, 'UL', node.get_cost()+1.4))
    b = [branch for branch in branches if branch.get_state() is not None]
    return b

def ActionMoveUp(state):
    state_copy = state.get_state()
    state_copy[1] = state_copy[1]+1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        return None
    else:
        # allowed_states.remove(state_copy)
        return state_copy
        
def ActionMoveUpRight(state):
    state_copy = state.get_state()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]+1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        return None
        # allowed_states.remove(state_copy)
    else:
        return state_copy

def ActionMoveRight(state):
    state_copy = state.get_state()
    state_copy[0] = state_copy[0]+1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy

def ActionMoveDownRight(state):
    state_copy = state.get_state()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]-1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy

def ActionMoveDown(state):
    state_copy = state.get_state().copy()
    state_copy[1] = state_copy[1]-1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy
    
def ActionMoveDownLeft(state):
    state_copy = state.get_state().copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]-1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy

def ActionMoveLeft(state):
    state_copy = state.get_state().copy()
    state_copy[0] = state_copy[0]-1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy

def ActionMoveUpLeft(state):
    state_copy = state.get_state().copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]+1
    if ((state_copy in obstacle_space) or (state_copy == state.get_parent_state())) :
        # allowed_states.remove(state_copy)
        return None
    else:
        return state_copy
    
def allowed_states_():
    
    for i in range(5,595,1):
        for j in range(5,245,1):
            # Adding uper rectangle
            if (95<i<155 and 0<j<105):
                obstacle_space.append([250-j,i])
                continue
            # Adding lower rectangle
            elif (95<i<155 and 155<j<250):
                obstacle_space.append([250-j,i])
                continue
            # Adding Hexagon 
            elif (((70*j-42*i-1750)<=0)& ((70*j+42*j-26950)<=0) & (i<=370) & ((70*j-42*i+9450)>=0)& ((70*j+42*j-15750)>=0) &(i>=230)):
                obstacle_space.append([250-j,i])
                continue
            # elif ((i<=370)&(i>=230)&(j>)):
            #     continue
            # Adding triangle
            elif((i>=455) & ((60*j+105*i-61575)<=0) & ((60*j-105*i+46575)>=0)):
                obstacle_space.append([250-j,i])
                continue
            else :
                allowed_states.append([250-j,i])
    
    # print(allowed_states)
    # exit(0)

def updateMapViz(canvas, state, color):
    X, Y, _ = canvas.shape
    # print(canvas.shape)
    transformed_y = X - state[0]
    transformed_x =  state[1] 
    canvas[transformed_y,transformed_x] = color
    # canvas[state[0], state[1]] = color
    return canvas

def addObstacles2Map(canvas):
    for i in obstacle_space:
        updateMapViz(canvas,i, [0,255,0])
    return canvas

def dijkstra(start_point, goal_point, result):
    nodes = queue.PriorityQueue()
    init_node = Node(start_point, None, None, 0)
    nodes.put((init_node.get_cost(), init_node))
    canvas_size = [250, 600]
    canvas = np.zeros([canvas_size[0], canvas_size[1], 3], dtype=np.uint8)
    canvas = updateMapViz(canvas, start_point, [0,255,255])
    canvas = updateMapViz(canvas, goal_point, [0,255,255])
    canvas = addObstacles2Map(canvas)
    # cv2.imshow("Canvas", canvas)
    # cv2.waitKey(0)
    

    goal_reached = False
    # moves_cost = {'U':1, 'UR':1.4, 'R':1, 'DR':1.4, 'D':1, 'DL':1.4, 'L':1, 'UL':1.4}
    #Correct assignment
    node_array = np.array([[Node([i,j], None, None, math.inf) for j in range(600)] for i in range(250)])
    
    # full_path = None
    print("Starting Algorithm.......")
    while (not nodes.empty()):
        
        current_node = nodes.get()[1]
        print(current_node.get_state())
        # print(canvas.shape)
        # exit(0)
        canvas = updateMapViz(canvas, current_node.get_state(), [0, 255, 255])
        canvas_video.append(canvas)
        result.write(canvas)
        cv2.imshow('Frame', canvas)
        cv2.waitKey(1)
        # print(current_node.get_state())

        if (current_node.get_state() == goal_point):
            print('Goal reached')
            print("The cost of path: ", current_node.get_cost())
            full_path, node_path = current_node.getFullPath()
            goal_reached = True
            for node in node_path:
                pos = node.get_state()
                canvas = updateMapViz(canvas, pos, [40, 40, 200])
                cv2.imshow('frame',canvas)
                # cv2.waitKey(5)
                cv2.waitKey(1)
                result.write(canvas)
                canvas_video.append(canvas)
            for i in range(600):
                canvas_video.append(canvas)
                result.write(canvas)
        else:
            moves = getBranches(current_node)
            # parent_cost = current_node.get_cost()
            for move in moves:
                child_state = move.get_state()
                new_cost = move.get_cost()
                if (node_array[child_state[1],child_state[0]].get_cost()==math.inf):
                    node_array[child_state[1],child_state[0]]=move
                    nodes.put((new_cost,move))
                else: 
                    if (new_cost < node_array[child_state[1], child_state[0]].get_cost()):
                        node_array[child_state[1], child_state[0]] = move
                        nodes.put((new_cost, move))
        if (goal_reached): 
            break
       

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--InitState",nargs='+', type=int, help = 'Initial state for the matrix')
    parser.add_argument("--GoalState",nargs='+', type=int, help = 'Goal state for the matrix')
    parser.add_argument('--SaveFolderName', default='./', help='Path to store generated files')

    allowed_states_()

    Args = parser.parse_args()
    result_folder = Args.SaveFolderName
    initial_point = Args.InitState
    goal_point = Args.GoalState
    initial_point = list(initial_point)
    goal_point = list(goal_point)

    if (initial_point not in allowed_states):
        print( "Start point in obstacle space or out of bound")

    if (goal_point not in allowed_states):
        print( "Goal point in obstacle space of out of bound")

    file_name = result_folder + "dijkstra_simulation.avi"
    # initial_point = 
    space_size = [250, 600]
    size_y, size_x = space_size
    result = cv2.VideoWriter(file_name, cv2.VideoWriter_fourcc(*'MJPG'), 300, (size_x, size_y))
    # space_map = updateCanvasVis(space_map, initial_point, [255,255,255])
    # space_map = updateCanvasVis(space_map, goal_point, [255,255,255])
    # space_map = addObstaclesVis(space_map)
    
    # canvas_size = [250,600]
    # canvas = np.zeros([canvas_size[0], canvas_size[1], 3], dtype=np.uint8)
    # for i in obstacle_space:
    #     canvas[i[0],i[1]] = [255,255,255]

    # # plt.imshow(canvas)
    # cv2.imshow('Frame',canvas)
    # # plt.show()
    # cv2.waitKey(0)
    
    
    
    
    print("Initial state is: ", initial_point)
    print("Goal state is: ", goal_point)

    start = timeit.default_timer()
    dijkstra(initial_point, goal_point, result)
    stop = timeit.default_timer()
    print('Time taken by Algorithm: ', stop-start)
    print("storing it to file ...")


    # Set up the video writer
    SaveFileName = 'dijkstra.avi'
    codec = cv2.VideoWriter_fourcc(*'MJPG')
    fps = 300
    size = (600, 250)
    video_writer = cv2.VideoWriter(SaveFileName, codec, fps, size)
    # Write each frame to the video
    for i in canvas_video:
        # Generate the next frame (replace with your own code)
        frame = i
        # Write the frame to the video
        video_writer.write(frame)
    # Release the video writer and close the file
    video_writer.release()

if __name__ == "__main__":
    main()