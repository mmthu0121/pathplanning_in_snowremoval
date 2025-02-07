import numpy as np
import random

maps = [[] for i in range(4)]

# name: map0
# type: matrix
# rows: 10
# columns: 10
maps[0] = [[9, 9, 0, 0, 0, 0, 0, 0, 0, 0],
           [9, 9, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 9, 9, 9, 0],
           [0, 0, 0, 0, 0, 0, 9, 9, 9, 9],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [2, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

# name: map1
# type: matrix
# rows: 14
# columns: 5
maps[1] = [[0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 9, 9, 9],
           [0, 0, 9, 9, 9],
           [0, 0, 9, 9, 9],
           [0, 0, 9, 0, 0],
           [0, 0, 9, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0],
           [0, 2, 0, 0, 0]]


# name: map2
# type: matrix
# rows: 12
# columns: 14
maps[2] = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 9, 9, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 9, 9, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 9, 9, 0, 0],
           [0, 0, 0, 0, 0, 9, 9, 9, 9, 0, 9, 9, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2]]



maps[3] = [[9, 9, 0, 0, 0, 0, 0, 0, 0, 0],
           [9, 9, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 2, 0, 0, 0, 9, 9, 9, 0],
           [0, 0, 0, 0, 0, 0, 9, 9, 9, 9],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

for i in range (0, len(maps)):
    print("i: {}".format(i))

    m = np.array(maps[i])
    with open("maps/map{}.npy".format(i), 'wb') as f:
        np.save(f, m)

    m = None
    # Load the map from file and print it
    with open("maps/map{}.npy".format(i), 'rb') as f:
        m = np.load(f)

    print(m)