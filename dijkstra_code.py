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
    
    # def __lt__(self,other):
    #     return self.getState() < other.getState()
    
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
    
def dijkstra(nodes, goal_point):

    canvas_size = [250,600]
    canvas = np.zeros([canvas_size[0], canvas_size[1], 3], dtype=np.uint8)

    goal_reached = False
    moves_cost = {'U':1, 'UR':1.4, 'R':1, 'DR':1.4, 'D':1, 'DL':1.4, 'L':1, 'UL':1.4}
    node_array = np.array([[Node([i,j], None, None, math.inf) for j in range(canvas_size[1])] for i in range(canvas_size[0])])

    full_path = None
    print("Starting Algirithm.......")
    while (not nodes.empty()):

        current_node = nodes.get()[1]

        if (current_node.getState() == goal_point):
            print('Goal reached')
            print("The cost of path: ", current_node.getCost())
            full_path, node_path = current_node.getFullPath()
            goal_reached = True

            # for node in node_path:
            #     pos = node.getState()
            break
        else:
            # find the moves from the current position
            moves = getBranches(current_node)
            parent_cost = current_node.get_cost()
            for move in moves:
                child_state = move.get_state()
                new_cost = parent_cost +moves_cost.get(move.get_move())
                if (node_array[child_state[0],child_state[1]]== math.inf):
                    child_node = Node(child_state, current_node,move,new_cost)
                    node_array[child_state[0],child_state[1]]=child_node
                    nodes.put((child_node.get_cost(),child_node))
                else: 
                    if (new_cost < node_array[child_state[0], child_state[1]].getCost()):
                        child_node = Node(child_state, current_node, move, new_cost)
                        node_array[child_state[0], child_state[1]] = child_node
                        nodes.put((child_node.getCost(), child_node))
                
        if (goal_reached): break
       

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
    
    # space_map = updateCanvasVis(space_map, initial_point, [255,255,255])
    # space_map = updateCanvasVis(space_map, goal_point, [255,255,255])
    # space_map = addObstaclesVis(space_map)

    nodes = queue.PriorityQueue()
    init_node = Node(initial_point, None, None, 0)
    nodes.put((init_node.getCost(), init_node))
    
    dijkstra(nodes, goal_point)
    
    print("Initial state is: ", initial_point)
    print("Goal state is: ", goal_point)
    print("Solving Puzzle ....")

    print("storing it to file ...")


if __name__ == "__main__":
    main()