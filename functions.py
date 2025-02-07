import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import random

def load_map(map_name):
    with open("maps/{}.npy".format(map_name), 'rb') as f:
        return np.load(f)

def clean_duplicate(path_add):
    index = 0
    while index < len(path_add)-2:
        if path_add[index] == path_add[index+1]:
            del path_add[index+1]
        index = index + 1
    return path_add

def snow_generate(matrix):
    matrix = [[9*(random.random()+0.111) if cell == 0 else cell for cell in row] for row in matrix]
    matrix = [[0 if cell == 2 or cell == 9 else cell for cell in row] for row in matrix]

    return matrix

def plot_snow(matrix, start, map_name):
    plt.figure()
    colors = [(1, 1, 1), (1, 0, 0)]  # White to Red
    n_bins = 100  # Number of bins for smooth transition
    cmap = LinearSegmentedColormap.from_list("black_to_white", colors, N=n_bins)

    #plt.imshow(matrix, cmap='gray', interpolation='nearest', vmin=0, vmax=10)
    im = plt.imshow(matrix, cmap= cmap, interpolation='nearest', vmin=0, vmax=10)
    plt.scatter(start[1], start[0], color='blue', label='Start', s=100, edgecolors='black', linewidths=2)

    plt.colorbar(im, label='Snow Depth')  # Add a colorbar for reference
    plt.title('Snow Depth Plot for ' + map_name)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')

    plt.legend(loc='upper center', bbox_to_anchor=(0, -0.05), ncol=1)
    # plt.show()

def plot_path_with_color_changes(matrix, start, path, heuristic, map_name):
    # plot_snow(matrix, start)

    # Separate the path into X and Y coordinates
    row, col = zip(*path)

    # List of colors to cycle through
    color_list = [
        'red', 'green', 'blue', 'purple', 'orange', 'cyan', 'magenta', 
        'yellow', 'brown', 'pink', 'lime', 'teal', 'navy', 'gray', 
        'violet', 'indigo', 'beige', 'coral', 'turquoise', 'salmon'
    ]
    color_index = 0
    colour = color_list[0]

    # Create a figure for the combined graph
    combined_fig, combined_ax = plt.subplots()
    combined_ax.set_title(map_name + " Combined Path Visualization for Type: "+heuristic.name)
    combined_ax.set_xlabel("X-axis")
    combined_ax.set_ylabel("Y-axis")
    combined_ax.grid(True)
    combined_ax.set_ylim(max(row)+1, -1)  # Reverse the Y-axis
    combined_ax.set_xlim(-1, max(col)+1)  # Adjust the X-axis limits

    # Create a list to store the segments for reverse plotting
    segments = []
    # plt.figure()

    # fig, ax = plt.subplots()

    # Iterate through the path, changing the color when it reaches the first point
    for i in range(1, len(path)):
        start = path[i-1]
        end = path[i]
        
        # ax.plot([start[1], end[1]], [start[0], end[0]], color=colour, marker='o', linestyle='-', markersize=4)
        
        # # Calculate the midpoint for the arrow
        # mid_x = (start[1] + end[1]) / 2
        # mid_y = (start[0] + end[0]) / 2

        # # Calculate the direction vector for the arrow
        # dx = (end[1] - start[1]) * 0.5  # Scale down for the arrow size
        # dy = (end[0] - start[0]) * 0.5

        # # Add an arrow in the middle of the line
        # ax.annotate(
        #     '', 
        #     xy=(mid_x + dx, mid_y + dy), 
        #     xytext=(mid_x - dx, mid_y - dy), 
        #     arrowprops=dict(arrowstyle='->', color=colour)
        # )

        # Check if the path returns to the first point
        if end == path[0] and i < len(path)-1:    
            color_index += 1  # Change the color index after every loop 
            colour = color_list[color_index % len(color_list)]
            # fig, ax = plt.subplots()
        
        segments.append((start, end, colour))
           
        # ax.set_title(f"Path Visualization {color_index + 1}")
        # ax.set_xlabel("X-axis")
        # ax.set_ylabel("Y-axis")
        # ax.grid(True)
        # ax.set_ylim(max(row) + 1, -1)  # Reverse the Y-axis
        # ax.set_xlim(-1, max(col) + 1)  # Adjust the X-axis limits

    
    for start, end, colour in reversed(segments):     
        combined_ax.plot([start[1], end[1]], [start[0], end[0]], color=colour, marker='o', linestyle='-', markersize=4)

    # Show the combined graph 
    plt.figure(combined_fig.number)
    # plt.show()  
