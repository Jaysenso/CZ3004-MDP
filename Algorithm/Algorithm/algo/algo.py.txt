import heapq
import math
from typing import List
import numpy as np
from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import Direction, MOVE_DIRECTION, TURN_FACTOR, ITERATIONS, TURN_RADIUS, SAFE_COST
from python_tsp.exact import solve_tsp_dynamic_programming

turn_wrt_big_turns = [[3 * TURN_RADIUS, TURN_RADIUS],
                  [4 * TURN_RADIUS, 2 * TURN_RADIUS]]


class pathFinder:
    def __init__(
            self,
            size_x: int,
            size_y: int,
            robot_x: int,
            robot_y: int,
            robot_direction: Direction,
            big_turn=None # the big_turn here is to allow 3-1 turn(0 - by default) | 4-2 turn(1)
    ):
        # Initialize a Grid object for the arena representation
        self.grid = Grid(size_x, size_y)
        # Initialize a Robot object for robot representation
        self.robot = Robot(robot_x, robot_y, robot_direction)
        # Create tables for paths and costs
        self.path_table = dict()
        self.cost_table = dict()
        if big_turn is None:
            self.big_turn = 0
        else:
            self.big_turn = int(big_turn)

    def add_obstacle(self, x: int, y: int, direction: Direction, obstacle_id: int):
        obstacle = Obstacle(x, y, direction, obstacle_id)
        self.grid.add_obstacle(obstacle)
        
    def reset_obstacles(self):
        self.grid.reset_obstacles()

    @staticmethod
    def compute_coord_distance(x1: int, y1: int, x2: int, y2: int, level=1):
        horizontal_distance = x1 - x2
        vertical_distance = y1 - y2

        # Euclidean distance
        if level == 2:
            return math.sqrt(horizontal_distance ** 2 + vertical_distance ** 2)

        return abs(horizontal_distance) + abs(vertical_distance)

    @staticmethod
    def compute_state_distance(start_state: CellState, end_state: CellState, level=1):
        return pathFinder.compute_coord_distance(start_state.x, start_state.y, end_state.x, end_state.y, level)

    @staticmethod
    def get_visit_options(n):
        s = []
        l = bin(2 ** n - 1).count('1')

        for i in range(2 ** n):
            s.append(bin(i)[2:].zfill(l))

        s.sort(key=lambda x: x.count('1'), reverse=True)
        return s

    def get_optimal_order_dp(self, retrying) -> List[CellState]:
        distance = 1e9
        optimal_path = []

        all_view_positions = self.grid.get_view_obstacle_positions(retrying)

        for op in self.get_visit_options(len(all_view_positions)):

            # Calculate optimal_cost table
            # Initialize `items` to be a list containing the robot's start state as the first item
            items = [self.robot.get_start_state()]
            # Initialize `cur_view_positions` to be an empty list
            cur_view_positions = []

            # For each obstacle
            for idx in range(len(all_view_positions)):
                # If robot is visiting
                if op[idx] == '1':
                    # Add possible cells to `items`
                    items = items + all_view_positions[idx]
                    # Add possible cells to `cur_view_positions`
                    cur_view_positions.append(all_view_positions[idx])
                    #print("obstacle: {}\n".format(self.grid.obstacles[idx]))

            # Generate the path cost for the items
            self.path_cost_generator(items)
            combination = []
            self.generate_combination(cur_view_positions, 0, [], combination, [ITERATIONS])

            for c in combination: # run the algo some times ->
                visited_candidates = [0] # add the start state of the robot

                cur_index = 1
                fixed_cost = 0 # the cost applying for the position taking obstacle pictures
                for index, view_position in enumerate(cur_view_positions):
                    visited_candidates.append(cur_index + c[index])
                    fixed_cost += view_position[c[index]].penalty
                    cur_index += len(view_position)
                
                cost_np = np.zeros((len(visited_candidates), len(visited_candidates)))

                for s in range(len(visited_candidates) - 1):
                    for e in range(s + 1, len(visited_candidates)):
                        u = items[visited_candidates[s]]
                        v = items[visited_candidates[e]]
                        if (u, v) in self.cost_table.keys():
                            cost_np[s][e] = self.cost_table[(u, v)]
                        else:
                            cost_np[s][e] = 1e9
                        cost_np[e][s] = cost_np[s][e]
                cost_np[:, 0] = 0
                _permutation, _distance = solve_tsp_dynamic_programming(cost_np)
                if _distance + fixed_cost >= distance:
                    continue

                optimal_path = [items[0]]
                distance = _distance + fixed_cost

                for i in range(len(_permutation) - 1):
                    from_item = items[visited_candidates[_permutation[i]]]
                    to_item = items[visited_candidates[_permutation[i + 1]]]

                    cur_path = self.path_table[(from_item, to_item)]
                    for j in range(1, len(cur_path)):
                        optimal_path.append(CellState(cur_path[j][0], cur_path[j][1], cur_path[j][2]))

                    optimal_path[-1].set_screenshot(to_item.screenshot_id)

            if optimal_path:
                # if found optimal path, return
                break

        return optimal_path, distance

    @staticmethod
    def generate_combination(view_positions, index, current, result, iteration_left):
        if index == len(view_positions):
            result.append(current[:])
            return

        if iteration_left[0] == 0:
            return

        iteration_left[0] -= 1
        for j in range(len(view_positions[index])):
            current.append(j)
            pathFinder.generate_combination(view_positions, index + 1, current, result, iteration_left)
            current.pop()

    def get_safe_cost(self, x, y):
        """Get the safe cost of a particular x,y coordinate wrt obstacles that are exactly 2 units away from it in both x and y directions

        Args:
            x (int): x-coordinate
            y (int): y-coordinate

        Returns:
            int: safe cost
        """
        for ob in self.grid.obstacles:
            if abs(ob.x-x) == 2 and abs(ob.y-y) == 2:
                return SAFE_COST
            
            if abs(ob.x-x) == 1 and abs(ob.y-y) == 2:
                return SAFE_COST
            
            if abs(ob.x-x) == 2 and abs(ob.y-y) == 1:
                return SAFE_COST

        return 0

    def get_neighbors(self, x, y, direction):  # TODO: see the behavior of the robot and adjust...
        neighbors = []
        # Assume that after following this direction, the car direction is EXACTLY md
        for dx, dy, md in MOVE_DIRECTION:
            if md == direction:  # if the new direction == md
                # Check for valid position
                if self.grid.reachable(x + dx, y + dy):  # go forward;
                    # Get safe cost of destination
                    safe_cost = self.get_safe_cost(x + dx, y + dy)
                    neighbors.append((x + dx, y + dy, md, safe_cost))
                # Check for valid position
                if self.grid.reachable(x - dx, y - dy):  # go back;
                    # Get safe cost of destination
                    safe_cost = self.get_safe_cost(x - dx, y - dy)
                    neighbors.append((x - dx, y - dy, md, safe_cost))

            else:  # consider 8 cases
                
                # Turning displacement is either 4-2 or 3-1
                bigger_change = turn_wrt_big_turns[self.big_turn][0]
                smaller_change = turn_wrt_big_turns[self.big_turn][1]

                # north <-> east
                if direction == Direction.NORTH and md == Direction.EAST:

                    # Check for valid position
                    if self.grid.reachable(x + bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        # Get safe cost of destination
                        safe_cost = self.get_safe_cost(x + bigger_change, y + smaller_change)
                        neighbors.append((x + bigger_change, y + smaller_change, md, safe_cost + 10))

                    # Check for valid position
                    if self.grid.reachable(x - smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        # Get safe cost of destination
                        safe_cost = self.get_safe_cost(x - smaller_change, y - bigger_change)
                        neighbors.append((x - smaller_change, y - bigger_change, md, safe_cost + 10))

                if direction == Direction.EAST and md == Direction.NORTH:
                    if self.grid.reachable(x + smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y + bigger_change)
                        neighbors.append((x + smaller_change, y + bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y - smaller_change)
                        neighbors.append((x - bigger_change, y - smaller_change, md, safe_cost + 10))

                # east <-> south
                if direction == Direction.EAST and md == Direction.SOUTH:
                    
                    if self.grid.reachable(x + smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y - bigger_change)
                        neighbors.append((x + smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y + smaller_change)
                        neighbors.append((x - bigger_change, y + smaller_change, md, safe_cost + 10))

                if direction == Direction.SOUTH and md == Direction.EAST:
                    if self.grid.reachable(x + bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y - smaller_change)
                        neighbors.append((x + bigger_change, y - smaller_change, md, safe_cost + 10))

                    if self.grid.reachable(x - smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y + bigger_change)
                        neighbors.append((x - smaller_change, y + bigger_change, md, safe_cost + 10))

                # south <-> west
                if direction == Direction.SOUTH and md == Direction.WEST:
                    if self.grid.reachable(x - bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y - smaller_change)
                        neighbors.append((x - bigger_change, y - smaller_change, md, safe_cost + 10))

                    if self.grid.reachable(x + smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y + bigger_change)
                        neighbors.append((x + smaller_change, y + bigger_change, md, safe_cost + 10))

                if direction == Direction.WEST and md == Direction.SOUTH:
                    if self.grid.reachable(x - smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y - bigger_change)
                        neighbors.append((x - smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x + bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y + smaller_change)
                        neighbors.append((x + bigger_change, y + smaller_change, md, safe_cost + 10))

                # west <-> north
                if direction == Direction.WEST and md == Direction.NORTH:
                    if self.grid.reachable(x - smaller_change, y + bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - smaller_change, y + bigger_change)
                        neighbors.append((x - smaller_change, y + bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x + bigger_change, y - smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + bigger_change, y - smaller_change)
                        neighbors.append((x + bigger_change, y - smaller_change, md, safe_cost + 10))

                if direction == Direction.NORTH and md == Direction.WEST:
                    if self.grid.reachable(x + smaller_change, y - bigger_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x + smaller_change, y - bigger_change)
                        neighbors.append((x + smaller_change, y - bigger_change, md, safe_cost + 10))

                    if self.grid.reachable(x - bigger_change, y + smaller_change, turn = True) and self.grid.reachable(x, y, preTurn = True):
                        safe_cost = self.get_safe_cost(x - bigger_change, y + smaller_change)
                        neighbors.append((x - bigger_change, y + smaller_change, md, safe_cost + 10))

        return neighbors

    def path_cost_generator(self, states: List[CellState]):
        """Generate the path cost between the input states and update the tables accordingly

        Args:
            states (List[CellState]): cell states to visit
        """
        def record_path(start, end, parent: dict, cost: int):

            # Update cost table for the (start,end) and (end,start) edges
            self.cost_table[(start, end)] = cost
            self.cost_table[(end, start)] = cost

            path = []
            cursor = (end.x, end.y, end.direction)

            while cursor in parent:
                path.append(cursor)
                cursor = parent[cursor]

            path.append(cursor)

            # Update path table for the (start,end) and (end,start) edges, with the (start,end) edge being the reversed path
            self.path_table[(start, end)] = path[::-1]
            self.path_table[(end, start)] = path

        def astar_search(start: CellState, end: CellState):
            # astar search algo with three states: x, y, direction

            # If it is already done before, return
            if (start, end) in self.path_table:
                return

            # Heuristic to guide the search: 'distance' is calculated by f = g + h
            # g is the actual distance moved so far from the start node to current node
            # h is the heuristic distance from current node to end node
            g_distance = {(start.x, start.y, start.direction): 0}

            # format of each item in heap: (f_distance of node, x coord of node, y coord of node)
            # heap in Python is a min-heap
            heap = [(self.compute_state_distance(start, end), start.x, start.y, start.direction)]
            parent = dict()
            visited = set()

            while heap:
                # Pop the node with the smallest distance
                _, cur_x, cur_y, cur_direction = heapq.heappop(heap)
                
                if (cur_x, cur_y, cur_direction) in visited:
                    continue

                if end.is_eq(cur_x, cur_y, cur_direction):
                    record_path(start, end, parent, g_distance[(cur_x, cur_y, cur_direction)])
                    return

                visited.add((cur_x, cur_y, cur_direction))
                cur_distance = g_distance[(cur_x, cur_y, cur_direction)]

                for next_x, next_y, new_direction, safe_cost in self.get_neighbors(cur_x, cur_y, cur_direction):
                    if (next_x, next_y, new_direction) in visited:
                        continue

                    move_cost = Direction.rotation_cost(new_direction, cur_direction) * TURN_FACTOR + 1 + safe_cost

                    # the cost to check if any obstacles that considered too near the robot; if it
                    # safe_cost =

                    # new cost is calculated by the cost to reach current state + cost to move from
                    # current state to new state + heuristic cost from new state to end state
                    next_cost = cur_distance + move_cost + \
                                self.compute_coord_distance(next_x, next_y, end.x, end.y)

                    if (next_x, next_y, new_direction) not in g_distance or \
                            g_distance[(next_x, next_y, new_direction)] > cur_distance + move_cost:
                        g_distance[(next_x, next_y, new_direction)] = cur_distance + move_cost
                        parent[(next_x, next_y, new_direction)] = (cur_x, cur_y, cur_direction)

                        heapq.heappush(heap, (next_cost, next_x, next_y, new_direction))

        # Nested loop through all the state pairings
        for i in range(len(states) - 1):
            for j in range(i + 1, len(states)):
                astar_search(states[i], states[j])


import math
import heapq
 
# Define the Cell class
class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell (g + h)
        self.g = float('inf')  # Cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination
 
# Define the size of the grid
ROW = 9
COL = 10
 
# Check if a cell is valid (within the grid)
def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)
 
# Check if a cell is unblocked
def is_unblocked(grid, row, col):
    return grid[row][col] == 1
 
# Check if a cell is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]
 
# Calculate the heuristic value of a cell (Euclidean distance to destination)
def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5
 
# Trace the path from source to destination
def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    row = dest[0]
    col = dest[1]
 
    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
 
    # Add the source cell to the path
    path.append((row, col))
    # Reverse the path to get the path from source to destination
    path.reverse()
 
    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()
 
def a_star_search_test(grid, src, dest):
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        print("Source or destination is invalid")
        return
    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return
    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return
 
    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]
 
    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j
 
    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))
 
    # Initialize the flag for whether destination is found
    found_dest = False
 
    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)
 
        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True
 
        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]
 
            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found!")
                    # Trace and print the path from source to destination
                    trace_path(cell_details, dest)
                    found_dest = True
                    return
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new
 
                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
 
    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")

#calculating Manhattan Distance between 2 points
def manhattan_distance(point1, point2):
    distance = 0
    for x1, x2 in zip(point1, point2):
        difference = x2 - x1
        absolute_difference = abs(difference)
        distance += absolute_difference

    return distance

if __name__ == "__main__":
    pass
