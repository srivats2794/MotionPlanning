
forward = [[-1,  0], # 0: go up
           [ 0, -1], # 1: go left
           [ 1,  0], # 2: go down
           [ 0,  1]] # 3: go right

# The car can perform 3 actions: 0: right turn, 1: no turn, 2: left turn
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 1] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

start = [4, 3, 0] #[grid row, grid col, direction]
                
goal = [2, 0] #[grid row, grid col]

heuristic = [[2, 3, 4, 5, 6, 7],
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]


def compute_plan(grid,start,goal,cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed=[[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    
    parent = [['       ' for row in range(len(grid[0]))] for col in range(len(grid))]

    plan =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]


    x = start[0]
    y = start[1]
    theta = start[2]
    g = 0
    h = heuristic[x][y]
    f = g+h
    open = [[f, g, h, x, y, theta]]
    
    found = False
    resign = False
    count = 0
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            
            x = next[3]
            y = next[4]
            theta = next[5]
            f = next[0]
            g = next[1]
            h = next[2]
           
            
            plan[x][y] = count
            count+=1
                        
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                j = 0
                for i in action:
                    
                    value = theta + i
                    if value<0:
                        theta2 = 3
                    elif value>3:
                        theta2 = 0
                    else:
                        theta2 = value
                    
                    
                    x2 = x + forward[theta2][0]
                    y2 = y + forward[theta2][1]                   
                    
                    
                    if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]):
                        
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost[j]
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2, theta2])
                            parent[x2][y2] = [x,y,theta]
                         
                    j += 1
                    
    return plan

def show(p):
    for i in range(len(p)):
        print p[i]
    return p
        
check = show(compute_plan(grid, start, goal, cost,heuristic))
