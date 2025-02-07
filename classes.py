import numpy as np
from enum import Enum, auto
from collections import deque

class PlannerStatus(Enum):
    STANDBY = auto()
    COVERAGE_SEARCH = auto()
    NEARST_UNVISITED_SEARCH = auto()
    FOUND = auto()
    NOT_FOUND = auto()
    RETURN_ORIGIN = auto()


class HeuristicType(Enum):
    MANHATTAN = auto()
    CHEBYSHEV = auto()
    VERTICAL = auto()
    HORIZONTAL = auto()


class CoveragePlanner():

    def __init__(self, map_open, snow_map=None):
        self.map_grid = map_open
        self.snow_map = snow_map

        # Add a new attribute to keep track of the step count
        self.max_steps = 15  # Maximum steps before resetting
        self.accu_snow = 0  # Accumulated snow
        self.max_accu_snow = 50  # Maximum accumulated snow before resetting

        self.start_pos = self.get_start_position()

        # Possible movements in x and y axis
        self.movement = [[-1,  0],  # up
                         [0, -1],    # left
                         [1,  0],    # down
                         [0,  1]]    # right

        # Readable description['up', 'left', 'down', 'right']
        self.movement_name = ['^', '<', 'v', '>']

        # Possible actions performed by the robot
        self.action = [-1, 0, 1, 2]
        self.action_name = ['R', '#', 'L', 'B']  # Right,Forward,Left,Backwards
        self.action_cost = [.2, .1, .2, .4]

        # A star movement cost
        self.a_star_movement_cost = [1, 1, 1, 1]

        # Trajectory list of points
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        # The grid that accumulate the visited positions in the map
        self.coverage_grid = np.copy(map_open)

        # The FSM variable
        self.state_ = PlannerStatus.STANDBY

        # Heuristic types of each search algorithm
        self.a_star_heuristic = HeuristicType.MANHATTAN
        self.cp_heuristic = HeuristicType.VERTICAL

        self.debug_level = -1

    def accumulate_snow(self, current_pos):
        self.accu_snow = self.accu_snow + self.snow_map[current_pos[0]][current_pos[1]]
        self.snow_map[current_pos[0]][current_pos[1]] = 0

    def find_shortest_path(self, grid, start, end, direction):

        #origin = self.get_start_position()
        #end = (origin[0], origin[1])
        visited = None
        rows, cols = len(grid), len(grid[0])
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        visited = set()
        queue = deque([(start, [start])])  # (current_position, path_so_far)
        
        while queue:
            (current, path) = queue.popleft()
            
            if current == end:
                return path  # Found the shortest path
            
            for d in directions:
                new_row, new_col = current[0] + d[0], current[1] + d[1]
                new_pos = (new_row, new_col)
                
                if direction == 0:
                    # Check if new position is within bounds, passable, and not visited
                    if (0 <= new_row < rows and
                        0 <= new_col < cols and
                        grid[new_row][new_col] != 9 and
                        #(grid[new_row][new_col] == 0 or grid[new_row][new_col] == 1 or grid[new_row][new_col] == 2 ) and 
                        new_pos not in visited):
                        
                        visited.add(new_pos)
                        queue.append((new_pos, path + [new_pos]))

                if direction == 1:
                    # Check if new position is within bounds, passable, and not visited
                    if (0 <= new_row < rows and
                        0 <= new_col < cols and
                        (grid[new_row][new_col] == 1 or grid[new_row][new_col] == 2 ) and 
                        new_pos not in visited):
                        
                        visited.add(new_pos)
                        queue.append((new_pos, path + [new_pos]))
        return []  # No path found

    # Print a given map grid with regular column width
    def print_map(self, m):
        for row in m:
            s = "["
            for i in range(len(m[0])):
                if type(row[i]) is str:
                    s += row[i]
                else:
                    s += "{:.1f}".format(row[i])
                if i is not (len(m[0])-1):
                    s += "\t,"
                else:
                    s += "\t]"
            print(s)


    # Return the initial x, y and orientation of the current map grid
    def get_start_position(self, orientation=0):
        for x in range(len(self.map_grid)):
            for y in range(len(self.map_grid[0])):
                if(self.map_grid[x][y] == 2):
                    return [x, y, orientation]

    def start(self, initial_orientation=0, a_star_heuristic=None, cp_heuristic=None):
        
        # Set current position to the given map start position
        self.current_pos = self.get_start_position(orientation=initial_orientation)

        self.coverage_grid = np.copy(self.map_grid)
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        if cp_heuristic is not None:
            self.cp_heuristic = cp_heuristic
        if a_star_heuristic is not None:
            self.a_star_heuristic = a_star_heuristic

        self.state_ = PlannerStatus.COVERAGE_SEARCH

    # Return the trajectory total cost
    def calculate_trajectory_cost(self, trajectory):
        cost = 0

        # Sum up the actions cost of each step
        for t in trajectory:
            if t[5] is not None:
                cost += self.action_cost[t[5]]
        return cost

    def get_xy_trajectory(self, trajectory):
        if type(trajectory) == list:
            if type(trajectory[0]) == list:
                return [t[1:3] for t in trajectory]
                # return np.array([t[1:3] for t in trajectory])
            return trajectory[1:3]
            # return np.array(trajectory[1:3])
        return []

    def result(self):
        found = self.state_ == PlannerStatus.FOUND
        total_steps = len(self.current_trajectory)-1
        xy_trajectory = self.get_xy_trajectory(self.current_trajectory)
        total_cost = self.calculate_trajectory_cost(self.current_trajectory)

        res = [found, total_steps, total_cost, self.current_trajectory, xy_trajectory]

        return res

    # Append the given trajectory to the main trajectory
    def append_trajectory(self, new_trajectory, algorithm_ref):
        if len(self.current_trajectory) > 0 and len(new_trajectory) > 0:
            new_trajectory[0][4] = self.current_trajectory[-1][4]

            # Add a special annotation to be shown at the policy map
            self.current_trajectory_annotations.append(
                [new_trajectory[0][1], new_trajectory[0][2], algorithm_ref])

            # remove the duplicated position
            self.current_trajectory.pop()
            
            self.step_count = len(new_trajectory)

        # Add the computed path to the trajectory list
        for t in new_trajectory:
            self.current_trajectory.append(t)

#---------------------------------------------------

    def coverage_search(self, initial_pos, heuristic):
        # Create a reference grid for visited coords
        closed = np.copy(self.coverage_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search",
                        "initial closed grid:", 2)
            print(closed)

        x = initial_pos[0]
        y = initial_pos[1]
        o = initial_pos[2]
        v = 0

        # Fill the initial coord in the iteration list
        trajectory = [[v, x, y, o, None, None, self.state_]]

        complete_coverage = False
        resign = False

        while not complete_coverage and not resign:

            if self.check_full_coverage(self.map_grid, closed):
                self.printd("coverage_search", "Complete coverage", 2)
                complete_coverage = True

            else:
                # Get the last visited coord info
                v = trajectory[-1][0]
                x = trajectory[-1][1]
                y = trajectory[-1][2]
                o = trajectory[-1][3]

                # [accumulated_cost, x_pos, y_pos, orientation, action_performed_to_get_here, next_action]
                possible_next_coords = []

                # calculate the possible next coords
                for a in range(len(self.action)):
                    o2 = (self.action[a]+o) % len(self.movement)
                    x2 = x + self.movement[o2][0]
                    y2 = y + self.movement[o2][1]

                    # Check if it is out of the map boundaries
                    if x2 >= 0 and x2 < len(self.map_grid) and y2 >= 0 and y2 < len(self.map_grid[0]):
                        # Check if this position was already visited or if it is a visitable position on the map
                        if closed[x2][y2] == 0 and self.map_grid[x2][y2] == 0:
                        #if closed[x2][y2] != 9 and self.map_grid[x2][y2] != 9 and closed[x2][y2] != 1 and self.map_grid[x2][y2] != 1:
                            # Compose the projected: actual accumulated cost + action cost + heuristic cost at given position
                            v2 = v + self.action_cost[a] + heuristic[x2][y2]
                            possible_next_coords.append(
                                [v2, x2, y2, o2, a, None, self.state_])

                # If there isn any possible next position, stop searching
                if len(possible_next_coords) == 0:
                    resign = True
                    self.printd("coverage_search",
                                "Could not find a next unvisited coord", 2)

                # otherwise update the trajectory list wiht the next position with the lowest possible cost
                else:
                    # rank by total_cost
                    possible_next_coords.sort(key=lambda x: x[0])

                    # update the last trajectory next_action
                    trajectory[-1][5] = possible_next_coords[0][4]

                    # add the lowest cost possible_next_coords to the trajectory list
                    trajectory.append(possible_next_coords[0])

                    # mark the chosen possible_next_coords position as visited
                    closed[possible_next_coords[0][1]
                           ][possible_next_coords[0][2]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search", "Heuristic: ", 2)
            print(heuristic)

            self.printd("coverage_search", "Closed: ", 2)
            print(closed)

            self.printd("coverage_search", "Policy: ", 2)
            self.print_policy_map(trajectory, [])

            self.printd("coverage_search", "Trajectory: ", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("coverage_search", "found: {}, total_steps: {}, total_cost: {}".format(
            not resign, total_steps, total_cost), 1)

        # Pack the standard response
        # trajectory:   [value , x, y, orientation, action_performed_to_get_here, next_action, current_state_]
        # res:          [Success?, trajectory, final_coverage_grid,total_cost_actions,total_steps]
        res = [not resign, trajectory, closed, total_cost, total_steps]

        return res

    # Find the shortest path between init and goal coords based on the A* Search algorithm
    def a_star_search_closest_unvisited(self, initial_pos, heuristic):

        # Create a reference grid for visited positions
        closed = np.zeros_like(self.map_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited",
                        "initial closed grid:", 2)
            print(closed)

        # Movement orientation at the A* visited positions
        orientation = np.full(
            (np.size(self.map_grid, 0), np.size(self.map_grid, 1)), -1)

        # Added the given A* initial position with its associated costs to the "open" list
        # "open" is a list of valid positions to expand: [[f, g, x, y]]
        # g: accumulated a* movement_cost (start it with 0 cost)
        # f: total cost = a*_movement_cost + heuristic_cost at given position
        # x,y: given position
        x = initial_pos[0]
        y = initial_pos[1]
        g = 0
        f = g + heuristic[x][y]
        open = [[f, g, x, y]]

        found = False  # Whether a unvisited position is found or not
        resign = False  # flag set if we can't find expand

        while not found and not resign:
            self.printd("a_star_search_closest_unvisited",
                        " open: {}".format(open), 2)

            # If there isn't any more positions to expand, the unvisited position could not be found, then resign
            if len(open) == 0:
                resign = True
                self.printd("a_star_search_closest_unvisited",
                            " Fail to find a path", 2)

            # Otherwise expand search again
            else:

                # Sort open positions list by total cost, in descending order and reverse it to pop the element with lowest total cost
                open.sort(key=lambda x: x[0])  # +heuristic[x[1]][x[2]])
                open.reverse()
                next = open.pop()

                # update current search x,y,g
                x = next[2]
                y = next[3]
                g = next[1]

                # Check if a unvisited position was found
                if self.coverage_grid[x][y] == 0:
                #if self.coverage_grid[x][y] < 1:
                    found = True
                else:
                    # calculate the possible next coords
                    for i in range(len(self.movement)):
                        x_next = x + self.movement[i][0]
                        y_next = y + self.movement[i][1]

                        # Check if it is out of the map boundaries
                        if x_next >= 0 and x_next < len(self.map_grid) and y_next >= 0 and y_next < len(self.map_grid[0]):
                            # Check if this position was already visited or if it is a visitable position on the map
                            if closed[x_next][y_next] == 0 and self.map_grid[x_next][y_next] == 0:
                            #if closed[x_next][y_next] != 9 and self.map_grid[x_next][y_next] != 9 and closed[x_next][y_next] != 1 and self.map_grid[x_next][y_next] != 1:
                                g2 = g + self.a_star_movement_cost[i]
                                f = g2 + heuristic[x_next][y_next]
                                open.append([f, g2, x_next, y_next])
                                closed[x_next][y_next] = 1
                                orientation[x_next][y_next] = i

        # initialize the trajectory
        trajectory = []

        # If path was found, then build the trajectory.
        # x and y at this point represents the last position on the search, which is the unvisited position
        if found:

            # Add the last position to the trajectory list, with both actions as None (they will be set later)
            trajectory = [
                [0, x, y, orientation[x][y], None, None, self.state_]]

            # Add the initial orientation to the orientation matrix
            orientation[initial_pos[0]][initial_pos[1]] = initial_pos[2]

            # Go backwards from the unvisited position founded to this search inital position
            # the 0 postfix means that this variable referes to the predecessor position, as 
            # this process below starts from the final position to the start position of the A*
            while (x != initial_pos[0] or y != initial_pos[1]):
                # Calculate the path predecessor position
                x0 = x - self.movement[orientation[x][y]][0]
                y0 = y - self.movement[orientation[x][y]][1]
                # The predecessor orientation is the one on the orientation matrix
                o0 = orientation[x0][y0]
                # The predecessor action will be set in the next iteration (its the next action of one position before it)
                a0 = None

                # Compute the required action index to get from the predecessor position to current iteration position
                a = (trajectory[-1][3]-o0 + 1) % len(self.action)

                # Update the successor position "action_performed_to_get_here" with the current next_action
                trajectory[-1][4] = a

                # Add the predecessor position and actions to the trajectory list
                trajectory.append([0, x0, y0, o0, a0, a, self.state_])

                # update x and y with the predecessor position, for next iteration
                x = x0
                y = y0

            trajectory.reverse()

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited", "Heuristic: ", 2)
            print(heuristic)

            self.printd("a_star_search_closest_unvisited", "Orientation: ", 2)
            print(orientation)

            self.printd("a_star_search_closest_unvisited", "Policy: ", 2)
            self.print_policy_map(trajectory, [])

            self.printd("[a_star_search_closest_unvisited", "Trajectory: ", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("a_star_search_closest_unvisited", "found: {}, total_steps: {}, total_cost: {}".format(
            found, total_steps, total_cost), 1)

        # Pack the standard response
        # trajectory:   [value , x, y, orientation, action_performed_to_get_here, next_action, current_state_]
        # res:          [Success?, trajectory, final_coverage_grid,total_cost_actions,total_steps]
        res = [found, trajectory, None, total_cost, total_steps]
        return res

    # Merge the two given grids and return True if all visitable positions were visited
    def check_full_coverage(self, grid, closed):
        return np.all(np.copy(grid)+np.copy(closed))

    # Return the Chebyshev heuristic at given target point
    def create_chebyshev_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = max(
                    abs(x - target_point[0]), abs(y - target_point[1]))
        return heuristic

    # Return the Manhattan heuristic at given target point
    def create_manhattan_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x - target_point[0])+abs(y - target_point[1])
        return heuristic

    # Return the horizontal heuristic at given target point
    def create_horizontal_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x - target_point[0])
        return heuristic

    # Return the vertical heuristic at given target point
    def create_vertical_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(y - target_point[1])
        return heuristic

    # Return the requested heuristic at the given target_point
    def create_heuristic(self, target_point, heuristic_type):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                if heuristic_type == HeuristicType.MANHATTAN:
                    heuristic[x][y] = abs(
                        x - target_point[0]) + abs(y - target_point[1])
                # elif heuristic_type == HeuristicType.CHEBYSHEV:
                #     heuristic[x][y] = max(
                #         abs(x - target_point[0]), abs(y - target_point[1]))
                elif heuristic_type == HeuristicType.HORIZONTAL:
                    heuristic[x][y] = abs(x - target_point[0])
                elif heuristic_type == HeuristicType.VERTICAL:
                    heuristic[x][y] = abs(y - target_point[1])
        return heuristic



    def compute(self):
        self.printd("compute", "{}".format(self.state_.name), 1)
        while self.compute_non_blocking():
            pass
        return self.state_

    # The finite state machine that will handle searching strategy
    # Returns True if search is not finished

    def compute_non_blocking(self):
        self.printd("compute_non_blocking", "{}".format(self.state_.name), 1)
        searching = False

        # Start the FSM by switching through the self.state_ attribute
        if self.state_ == PlannerStatus.COVERAGE_SEARCH:

            # Search using coverage_search algorithm
            heuristic = self.create_heuristic(
                self.current_pos, self.cp_heuristic)
            res = self.coverage_search(self.current_pos, heuristic)

            # Update the current position to the final search position
            self.current_pos = [res[1][-1][1], res[1][-1][2], res[1][-1][3]]

            self.append_trajectory(res[1], "CS")

            # Update the current coverage_grid
            self.coverage_grid = res[2]

            # Check if path was successfully found. If not, try to find the closest unvisited position
            if res[0]:
                self.state_ = PlannerStatus.FOUND
                self.current_trajectory[-1][6] = PlannerStatus.FOUND
            else:
                self.state_ = PlannerStatus.NEARST_UNVISITED_SEARCH
                searching = True

        elif self.state_ == PlannerStatus.NEARST_UNVISITED_SEARCH:

            # Search using a_star_search_closest_unvisited algorithm
            heuristic = self.create_heuristic(
                self.current_pos, self.a_star_heuristic)
            res = self.a_star_search_closest_unvisited(
                self.current_pos, heuristic)

            # In case a path was found
            if res[0]:
                # Update the current position to the final search position
                self.current_pos = [res[1][-1][1],
                                    res[1][-1][2], res[1][-1][3]]

                self.append_trajectory(res[1], "A*")

                # Set FSM to do a coverage search again
                self.state_ = PlannerStatus.COVERAGE_SEARCH
                searching = True

            # If no path was found, just finish searching
            else:
                self.state_ = PlannerStatus.NOT_FOUND
                if len(self.current_trajectory) > 0:
                    self.current_trajectory[-1][6] = PlannerStatus.NOT_FOUND

        else:
            self.printd("compute_non_blocking",
                        "Invalid state given, stop FSM", 0)

        return searching

    # Print helper function with the standarized printing structure
    # [function_name] message
    def printd(self, f, m, debug_level=0):
        if debug_level <= self.debug_level:
            print("["+f+"] "+m)

    # Set the debug level
    # Determine how much information is going to be shown in the terminal
    def set_debug_level(self, level):
        self.debug_level = level