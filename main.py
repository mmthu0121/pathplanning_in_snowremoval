import os
import csv
import numpy as np
from tabulate import tabulate
import matplotlib.pyplot as plt

from functions import load_map, clean_duplicate, snow_generate, plot_snow, plot_path_with_color_changes
from classes import CoveragePlanner, HeuristicType

os.system('cls')  # Use 'cls' if on Windows

test_show_each_result = False

maps = ["map0", "map1"]
orientations = [0, 1, 2, 3]    
cp_heuristics = [ HeuristicType.VERTICAL, HeuristicType.HORIZONTAL,
                 HeuristicType.MANHATTAN] #HeuristicType.CHEBYSHEV, 
astar_heuristics = [HeuristicType.VERTICAL, HeuristicType.HORIZONTAL,
                 HeuristicType.MANHATTAN] #HeuristicType.CHEBYSHEV, 

for map_name in maps:
    compare_tb = []

    target_map = load_map(map_name)
    path_map = load_map(np.copy(map_name))

    snow_map = np.copy(path_map)
    snow_map = snow_generate(snow_map)

    cp = CoveragePlanner(target_map, snow_map)    #cp.map_grid

    origin = cp.get_start_position()
    origin = (origin[0], origin[1])

    print("Path Planning for Snow Removal Robot\n")
    print("Snow Depth Map")
    cp.print_map(cp.snow_map)
    snow_map_shown = np.copy(cp.snow_map)

    print("\n")
    plot_snow(snow_map_shown, origin, map_name)

    for heuristic in cp_heuristics:
        
        path_add = [(origin[0], origin[1])]
        index = 0
        
        # for a_star in astar_heuristics:
        a_star = heuristic
        for orientation in orientations:
            cp.start(initial_orientation=orientation, cp_heuristic=heuristic, a_star_heuristic=a_star)
            cp.compute()

            res = [heuristic.name, orientation]
            res.extend(cp.result())
            compare_tb.append(res)

        compare_tb.sort(key=lambda x: (x[2], x[3], x[4]))

        summary = [row[0:4] for row in compare_tb]
        for row in summary:
            row[1] = cp.movement_name[row[1]]

        compare_tb_headers = ["Heuristic",
                            "Orientation", "Found?", "Steps"]
        summary_tb = tabulate(summary, compare_tb_headers,
                            tablefmt="pretty", floatfmt=".2f")

        
        print("Starting Point: ", path_add)
        print("\n")

        print("----------------------------------------------------------------------------------------------------------")
        print("\n")

        created_path = compare_tb[0][6]
        path = []

        for item in range(len(created_path)):
            items = (created_path[item][0], created_path[item][1])
            path.append(items)


        while len(path) > 2:
            forward_path = []
            if len(path) < cp.max_steps+1 and cp.accu_snow < cp.max_accu_snow*0.9:
                for i in range(len(path)-1):
                    if cp.accu_snow < cp.max_accu_snow*0.9:
                        startP = path[i]
                        endP = path[i+1]
                        go_path = cp.find_shortest_path(target_map, startP, endP, 0)
                        cp.accumulate_snow(endP)
                        for i in go_path:
                            path_add.append(i)
                            forward_path.append(i)
            else:
                for i in range(cp.max_steps):
                    if cp.accu_snow < cp.max_accu_snow*0.9:
                        startP = path[i]
                        endP = path[i+1]
                        go_path = cp.find_shortest_path(target_map, startP, endP, 0)
                        cp.accumulate_snow(endP)
                        for i in go_path:
                            path_add.append(i)
                            forward_path.append(i)
            
            forward_path = clean_duplicate(forward_path)
            print("Shortest Path from Starting Point:", forward_path)

            for point in path_add:
                if path_map[point[0]][point[1]] == 2:
                    path_map[point[0]][point[1]] = 2
                else:
                    path_map[point[0]][point[1]] = 1

            return_path = cp.find_shortest_path(path_map, path_add[-1], path_add[0], 1)
            
            for i in range(len(return_path)-1):
                path_add.append(return_path[i])

            path_add.append(origin)

            print("Shortest Path back to Starting Point:", return_path)
            print("\n")
            print("Accumulated Snow: ",cp.accu_snow)
            print("\n")

            cp.accu_snow = 0

            path = [item for item in path if item not in path_add]
            path = [origin] + path

            while index < len(path_add)-2:
                if path_add[index] == path_add[index+1]:
                    del path_add[index+1]
                index = index + 1

            print("Current Map Condition (0: Snow, 1: Cleared, 2: Start Point, 9: Obstacle)")
            cp.print_map(path_map)
            print("\n")
            print("----------------------------------------------------------------------------------------------------------")
            #print("\n")
            #cp.print_map(cp.coverage_grid)
            print("\n")

        print("Final Path:", path_add)
        print("\n")
        print("Total Steps of the best path: {}".format(len(path_add)))
        print("\n\n")

        #plot_snow(snow_map_shown, origin)

        with open('path_{}_{}.csv'.format(heuristic.name, map_name), 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(path_add)

        plot_path_with_color_changes(snow_map_shown, origin, path_add, heuristic, map_name)
plt.show()

"""
    i = len(compare_tb[0][6])

    #print(i)
    
    turn = int(i / 6)
    route = [[] for i in range(turn)]

    for i in range(1, turn):  
        for j in range(0, 6*i):
            route[i-1].append(compare_tb[0][6][j])
        for k in range(0, 6*i):
            l = 6*i - k
            route[i-1].append(compare_tb[0][6][l])

    #print(route)

    for element in route:
        print(element)

    print("\nTotal Steps of the best path: {}".format(len(route)))
    print("\n\n")
"""

"""
    path = [(9, 0), (9, 1), (9, 2), (9, 3), (9, 4), 
            (9, 5), (9, 6), (9, 7), (9, 8), (9, 9), 
            (8, 9), (8, 8), (8, 7), (8, 6), (8, 5), 
            (8, 4), (8, 3), (8, 2), (8, 1), (8, 0), 
            (7, 0), (6, 0), (5, 0), (4, 0), (3, 0), 
            (2, 0), (2, 1), (3, 1), (4, 1), (5, 1), 
            (6, 1), (7, 1), (7, 2), (7, 3), (7, 4), 
            (7, 5), (7, 6), (7, 7), (7, 8), (7, 9), 
            (6, 9), (6, 8), (6, 7), (6, 6), (6, 5), 
            (6, 4), (6, 3), (6, 2), (5, 2), (4, 2), 
            (3, 2), (2, 2), (1, 2), (0, 2), (0, 3), 
            (1, 3), (2, 3), (3, 3), (4, 3), (5, 3), 
            (5, 4), (5, 5), (4, 5), (4, 4), (3, 4), 
            (2, 4), (1, 4), (0, 4), (0, 5), (1, 5), 
            (2, 5), (3, 5), (3, 6), (3, 7), (3, 8), 
            (3, 9), (4, 9), (3, 9), (2, 9), (1, 9), 
            (0, 9), (0, 8), (1, 8), (2, 8), (2, 7), 
            (2, 6), (1, 6), (1, 7), (0, 7), (0, 6)]
    """