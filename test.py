    # Find the shortest path between init and goal coords based on the A* Search algorithm
    def a_star_search_closest_unvisited(self, initial_pos, heuristic):

        # Create a reference grid for visited positions
        closed = np.zeros_like(self.map_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

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
            # If there isn't any more positions to expand, the unvisited position could not be found, then resign
            if len(open) == 0:
                resign = True

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