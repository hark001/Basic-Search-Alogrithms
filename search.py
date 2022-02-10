# Basic searching algorithms

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node
        self.traverse_directions = 0 #directions traversed so far

def trace_path(path,current_node):
    parent_node = current_node.parent
    path.append([current_node.row,current_node.col])
    while(parent_node != None):
        path = [[parent_node.row,parent_node.col]] + path
        parent_node = parent_node.parent
    return path

def get_traverse_direction(current_node):
    return [[current_node.row, current_node.col +1], [current_node.row +1, current_node.col], [current_node.row, current_node.col -1], [current_node.row -1, current_node.col]]

def is_start_node_obstacle(grid,start):
    if grid[start[0]][start[1]] == 1:
        return True
    else:
        return False

def get_l1_dist(current_node, goal):
    l1_dist = abs(current_node.row - goal[0]) + abs(current_node.col - goal[1])
    return l1_dist

def update_cost_come(current_node, new_node):
    g = current_node.g + abs(current_node.row - new_node.row) + abs(current_node.col - new_node.col)
    return g


def explore_nodes(grid, open_list, steps,path, goal, visited_list, mode):
    found = False
    while (len(open_list) != 0):
        current_node = open_list.pop(0)
        if([current_node.row,current_node.col] in visited_list):
            continue
        steps += 1
        visited_list.append([current_node.row,current_node.col])
        
        if  goal == [current_node.row,current_node.col]:
            found = True
            path = trace_path(path,current_node)
            break
        
        #Exploring right, down, left, up
        traverse_direction = get_traverse_direction(current_node)
        
        if mode == "dfs":
            traverse_direction.reverse()
        
        for [new_node_row, new_node_col] in traverse_direction :
            #Checking if the row exits
            if ( 0 <= new_node_row < len(grid)):
                
                #Checking if the col exists
                if ( 0 <= new_node_col < len(grid[current_node.row])):
                    if (grid[new_node_row][new_node_col] == 0):
                        new_node = Node(new_node_row, new_node_col, grid[new_node_row][new_node_col], None)
                        new_node.parent = current_node

                        if mode =="dfs":
                            open_list.insert(0,new_node)
                        else:
                            if mode == "dijk":
                                new_node.g = update_cost_come(current_node, new_node)
                            elif mode == "astar":
                                new_node.g = update_cost_come(current_node, new_node)
                                new_node.h = get_l1_dist(new_node, goal)
                                new_node.cost = new_node.g + new_node.h
                                                        
                            if [new_node.row, new_node.col] not in visited_list:
                                open_list.append(new_node)
        if mode =="dijk":
            open_list = sorted(open_list, key=lambda x: x.g)
        elif mode =="astar":
            open_list = sorted(open_list, key=lambda x: x.cost)
    return path, steps, found

def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    open_list = []
    visited_list = []
    if is_start_node_obstacle(grid,start) == False:
        starting_node = Node(start[0],start[1],grid[start[0]][start[1]],None)
        open_list.append(starting_node)
        path, steps, found = explore_nodes(grid, open_list, steps,path, goal, visited_list, "bfs")

        if found:
            print(f"It takes {steps} steps to find a path using BFS")
        else:
            print("No path found")
    
    else:
        print("[ Error: Starting Node cannot be an obstacle]")
    
    return path, steps

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    open_list = []
    visited_list = []
    
    if is_start_node_obstacle(grid,start) == False:
        starting_node = Node(start[0],start[1],grid[start[0]][start[1]],None)
        open_list.append(starting_node)
        path, steps, found = explore_nodes(grid, open_list, steps,path, goal, visited_list, "dfs")
        if found:
            print(f"It takes {steps} steps to find a path using DFS")
        else:
            print("No path found")
    
    else:
        print("[ Error: Starting Node cannot be an obstacle]")
    
    return path, steps

def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    open_list = []
    visited_list = []
    if is_start_node_obstacle(grid,start) == False:
        starting_node = Node(start[0],start[1],grid[start[0]][start[1]],None)
        starting_node.g = 0
        open_list.append(starting_node)
        path, steps, found = explore_nodes(grid, open_list, steps,path, goal, visited_list, "dijk")

        if found:
            print(f"It takes {steps} steps to find a path using Dijkstra")
        else:
            print("No path found")
    else:
        print("[ Error: Starting Node cannot be an obstacle]")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    open_list = []
    visited_list = []
    if is_start_node_obstacle(grid,start) == False:
        starting_node = Node(start[0],start[1],grid[start[0]][start[1]],None)
        starting_node.g = 0
        starting_node.h = 0
        starting_node.cost = starting_node.g + starting_node.h
        open_list.append(starting_node)
        path, steps, found = explore_nodes(grid, open_list, steps,path, goal, visited_list, "astar")
        if found:
            print(f"It takes {steps} steps to find a path using A*")
        else:
            print("No path found")
    else:
        print("[ Error: Starting Node cannot be an obstacle ]")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()