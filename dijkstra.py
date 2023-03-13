import numpy as np
import time
import cv2                     ####### importing the necessary libraries is a must.
from queue import PriorityQueue

# Define the move functions
move = {"up": (0, -1, 1),
        "down": (0, 1, 1),
        "left": (-1, 0, 1),
        "right": (1, 0, 1),
        "up_left": (-1, -1, np.sqrt(2)),
        "up_right": (1, -1, np.sqrt(2)),
        "down_left": (-1, 1, np.sqrt(2)),
        "down_right": (1, 1, np.sqrt(2))
}

#  Defining the obstacle function which returns true if the given point (x,y) is inside an obstacle or not
def obstacle(x, y):
    rectanglefirst = (0 <= y <= 100 and 100 <= x <= 150)
    rectanglesecond = (150 <= y <= 250 and 100 <= x <= 150)
    hexagon = ((75/2) * abs(x-300)/75 + 50 <= y <= 250 - (75/2) * abs(x-300)/75 - 50 and 225 <= x <= 375)
    triangle = ((200/100) * (x-460) + 25 <= y <= (-200/100) * (x-460) + 225 and 460 <= x <= 510)
    return triangle or rectanglefirst or rectanglesecond or hexagon 

width, height, clearance = 600, 250, 5
obs_map = np.full((height, width, 3), 255, dtype=np.uint8)       ###### defining the obstacle map with width, height and clearance

for x in range(width):
    for y in range(height):
        if obstacle(x, y):
            obs_map[y, x] = (0, 0, 0)
        elif obstacle(x, y + clearance) or obstacle(x + clearance, y) or obstacle(x + clearance, y + clearance):
            obs_map[y, x] = (255,0,0)
        elif obstacle(x, y - clearance) or obstacle(x - clearance, y) or obstacle(x - clearance, y - clearance):
            obs_map[y, x] = (0, 255, 0)
        elif obstacle(x + clearance, y - clearance) or obstacle(x - clearance, y + clearance):
            obs_map[y, x] = (0, 0, 255)
            
###3###For each point in the map,check if the point is inside an obstacle or not. 

def is_free(node):
    x, y = node                                              ######Define a function to check if a node is free
    in_map = (0 <= x < width and 0 <= y < height)
    return in_map and obs_map[y, x][0] == 255

def is_goal(current_node, goal_node):                          ########Define a function to check if the current node is the goal node
    return current_node == goal_node

class Dijkstra:                      
                                                    
    def __init__(self, start_node, goal_node):
        self.start_node = start_node
        self.goal_node = goal_node
        self.cost_to_come = {start_node: 0}
        self.cost = {start_node: 0}
        self.parent = {start_node: None}          ########Define the Dijkstra class with the start and goal node as the constructor input
        self.start_node = start_node
        self.open_list = PriorityQueue()
        self.open_list.put((0, start_node))
        self.closed_list = set()
        self.visited = set()
        self.visited.add(start_node)

    def expand_node(self, current_node):
        for action in move.keys():                                    ########Define the variables to keep track of the cost to come, cost, parent, open and closed lists, and visited nodes
            new_node, move_cost = self.move(current_node, action)
            if is_free(new_node) and new_node not in self.closed_list:
                new_cost_to_come = self.cost_to_come[current_node] + move_cost
                if new_node not in self.cost_to_come or new_cost_to_come < self.cost_to_come[new_node]:
                    self.cost_to_come[new_node] = new_cost_to_come
                    self.cost[new_node] = new_cost_to_come
                    self.parent[new_node] = current_node
                    self.open_list.put((new_cost_to_come, new_node))
                    self.visited.add(new_node)
    
    def move(self, current_node, action):
        x, y = current_node
        dx, dy, cost = move[action]
        return (x + dx, y + dy), cost

    def search(self):                                        #######Define the search function that finds the optimal path from the start to the goal node
        while not self.open_list.empty():
            current_cost, current_node = self.open_list.get()
            self.closed_list.add(current_node)
            if is_goal(current_node, self.goal_node):
                return self.backtrack()
            self.expand_node(current_node)
        return None
    
    def backtrack(self):                                      #######Define the backtrack function that backtracks from the goal node to the start node using the parent nodes
        current_node = self.goal_node
        path = [current_node]
        while current_node != self.start_node:
            current_node = self.parent[current_node]
            path.append(current_node)
        return path[::-1]


def animate(visited, path):
    for node in visited:                        #### #Define the animate function that displays the path in different colors on the obstacle map using OpenCV
        x, y = node
        obs_map[y, x] = (0, 0, 255)
        cv2.imshow("Map", obs_map[::-1, :, :])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for node in path:
        x, y = node
        obs_map[y, x] = (0, 255, 0)
        cv2.imshow("Map", obs_map[::-1, :, :])
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()           ####cv2 






# voiding for valid start and goal nodes from client
while True:
    start_str = input("\nGive the start node (in the format x,y): ")
    start_node = tuple(map(int, start_str.split(',')))
    goal_str = input("Give the goal node (in the format x,y): ")
    goal_node = tuple(map(int, goal_str.split(',')))

    # Check if start and goal nodes are valid
    if not is_free(start_node):
        print("\nError: Start node is not responding.")
        continue
    elif not is_free(goal_node):
        print("\nError: Goal node is not responding.")
        continue
    break





# Running Dijkstra's Algorithm
start_time = time.time()
dijkstra = Dijkstra(start_node, goal_node)
path = dijkstra.search()
end_time = time.time()



# Printing the  results
if path is None:
    print("\nError: No path found.")
else:
    print("\nPlease enjoy the visualization. Path found in {:.4f} seconds.".format(end_time - start_time))
    print("Path length: {}".format(len(path)))
    print("Number of nodes visited as per : {}".format(len(dijkstra.visited)))
    animate(dijkstra.visited, path)
