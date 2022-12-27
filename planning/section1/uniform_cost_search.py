from queue import Queue,PriorityQueue
import numpy as np
from enum import Enum

class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
    
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])
            
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
        valid.remove(Action.RIGHT)
        
    return valid

def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str_)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'
    
    pos = start
    
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'  
    return sgrid

def uniform_cost(grid, start, goal):

    # TODO: Initialize the starting variables
    path = []
    queue = PriorityQueue()
    visited = set()
    
    branch = {}
    found = False
    
    queue.put((0,start))
    visited.add(start)
    
    while not queue.empty():
        # TODO: Remove the first element from the queue
        print(queue.queue)
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        # TODO: Check if the current vertex corresponds to the goal state
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            valid_acts =  valid_actions(grid, current_node)
            print("len of valid actions: ", len(valid_acts))
            for action in valid_acts:
                # TODO: determine the next_node using the action delta
                next_node = (current_node[0] + action.delta[0], current_node[1] + action.delta[1])
                # TODO: compute the new cost
                new_cost = current_cost + action.cost
                
                # TODO: Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1. Mark it as visited
                # 2. Add it to the queue
                if next_node not in visited:
                    queue.put((new_cost,next_node))
                    visited.add(next_node)
                    branch[next_node] = (new_cost,current_node,action)
    #print(branch)         
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
            
    return path[::-1], path_cost

def main():
    # Define a start and goal location
    start = (0, 0)
    goal = (4, 4)
        # Define your grid-based state space of obstacles and free space
    grid = np.array([
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 0, 0, 1, 1, 0],
        [0, 0, 0, 1, 0, 0],
    ])

    path, path_cost = uniform_cost(grid, start, goal)
    #print(path_cost, path)
    print(visualize_path(grid, path, start))


main()