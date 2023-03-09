import numpy as np
import argparse
import itertools
import math
import queue

# Node class 
class Node:
    # id_obj = itertools.count()
    def __init__(self, state, parent, move, cost): 
        self.state = state
        self.parent = parent
        # if parent is None:
        #    self.parent_idx=0
        # else:
        #     self.parent_idx=parent.get_index()
        # self.Index = next(Node.id_obj)
        self.move = move
        self.cost = cost

    def get_state(self):
        return self.state
    def get_parent(self):
        return self.parent
    # def get_index(self):
    #     return self.Index
    # def get_parent_index(self):
    #     return self.parent_idx
    def get_move(self):
        return self.move
    def get_cost(self):
        return self.cost
    
    def getFullPath(self):
        moves = []
        nodes = []
        current_node = self
        while(current_node.getMove() is not None):
            moves.append(current_node.getMove())
            nodes.append(current_node)
            current_node = current_node.getParent()
        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        return moves, nodes
    
    def __lt__(self,other):
        return self.getState() < other.getState()
    

def ActionMoveUp(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveUpRight(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]+1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveRight(state):
    state_copy = state.copy()
    state_copy[1] = state_copy[1]+1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveDownRight(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]+1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveDown(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]-1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None
    
def ActionMoveDownLeft(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]-1
    state_copy[1] = state_copy[1]-1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveLeft(state):
    state_copy = state.copy()
    state_copy[1] = state_copy[1]-1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None

def ActionMoveUpLeft(state):
    state_copy = state.copy()
    state_copy[0] = state_copy[0]+1
    state_copy[1] = state_copy[1]-1
    if state_copy in allowed_states:
        return state_copy
    else:
        return None
    

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
    file_name = result_folder + "dijkstra_simulation.avi"
    canvas_size = [250,600]

    canvas = np.zeros([canvas_size[0], canvas_size[1], 3], dtype=np.uint8)
    space_map = updateCanvasVis(space_map, initial_point, [255,255,255])
    space_map = updateCanvasVis(space_map, goal_point, [255,255,255])
    space_map = addObstaclesVis(space_map)

    nodes = queue.PriorityQueue()
    init_node = Node(initial_point, None, None, 0)
    nodes.put((init_node.getCost(), init_node))


    print("Initial state is: ", initial_point)
    print("Goal state is: ", goal_point)
    print("Solving Puzzle ....")

    print("storing it to file ...")


if __name__ == "__main__":
    main()