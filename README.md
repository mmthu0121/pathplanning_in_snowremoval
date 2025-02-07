# Path Planning Algorithm for Snow Removal
# **Pathfinding in Snow**

## **Table of Contents**
- [Introduction](#introduction)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Usage](#usage)
- [Algorithm Overview](#algorithm-overview)
- [Future Enhancements](#future-enhancements)
- [Contributors](#contributors)
- [License](#license)

---

## **Introduction**
This project provides an optimal pathfinding algorithm for snow removal in a grid-based environment. The algorithm minimizes the distance traveled by a snowplow, considering constraints like snow-carrying capacity and the need to revisit cleared paths when returning to the origin.

The goal is to develop efficient snow-clearing strategies using heuristic-based approaches, which can be applied in urban planning or autonomous vehicle systems.

---

## **Features**
- Implements three heuristic algorithms for pathfinding:
  - **Vertical Heuristic**: Clears snow column by column.
  - **Horizontal Heuristic**: Clears snow row by row.
  - **Manhattan Heuristic**: Uses Manhattan distance for optimized pathfinding.
- Simulates snow accumulation and clearing.
- Handles return-to-origin behavior when capacity is exceeded.
- Provides visual output for the snow-clearing process.
- Easily configurable grid and parameters.

---

## **Prerequisites**
Before running the project, ensure the following are installed:
- Python 3.8 or higher
<!-- - Required Python libraries (see `requirements.txt` for the full list) -->

---

## **Setup**
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/mmthu0121/pathplanning_in_snowremoval.git
   cd pathplanning_in_snowremoval

2. **Prepare the Snow Map**:  
   - Open the `generate_map.py` file located in the project folder.  
   - Define your snow-covered area by inputting a grid, where:  
     - Each cell value represents the map (e.g., `0` = snow, `2` = start point, `9` = obstacle).  
     - Rows are separated by newlines, and columns are separated by spaces.  
   - Example format for a 5x5 grid:  
     ```
     0 0 0 0 0
     0 9 9 0 0
     0 9 9 0 0
     2 0 0 0 0
     0 0 0 0 0
     ```
   - Save the file after making changes.

---

## **Usage**
1. **Run the Main Script**:  
   Execute the main Python script to start the snow-clearing process:
   ```bash
   python main.py

2. **View the Output**:  
   - After running the script, the console will display the snow-clearing process step by step, showing how the snowplow moves across the grid and clears the snow.  
   - If you have enabled the visualization, a graphical representation will be shown, illustrating the paths taken by the snowplow, the snow accumulation, and the cleared areas.
   - You will also see the remaining snow on the grid and the current status of the snowplow (whether it’s moving, clearing, or returning to the origin).

3. **Configure Parameters**:  
   - To customize the snowplow's behavior, edit the `classes.py` file.  
   The following parameters can be configured:
     - **Snowplow Capacity**: Set the maximum amount of snow the snowplow can carry before it needs to return to the origin for unloading.
     - **Heuristic Algorithm**: Choose the algorithm for pathfinding (options include **Vertical**, **Horizontal**, or **Manhattan**).
     - **Snow Map**: Update the file to define the snow distribution in the grid. The grid values represent snow depth, where `0` is no snow and higher numbers indicate more snow.
   - Save the file after making any changes.

---

## **Algorithm Overview**
The project uses heuristic algorithms to determine the optimal path for the snowplow. The goal is to clear the snow in the most efficient way possible, considering the snowplow's capacity and the grid configuration. The available heuristics are:

- **Vertical Heuristic**:  
  This algorithm clears the snow column by column, moving vertically down each column before proceeding to the next. It’s best suited for grids with vertical snow distributions.

- **Horizontal Heuristic**:  
  In this approach, the snowplow moves row by row, clearing one row before moving to the next. This heuristic works best for horizontally aligned snow distributions.

- **Manhattan Heuristic**:  
  The Manhattan heuristic calculates the Manhattan distance between points on the grid. The snowplow moves in a way that minimizes the total distance traveled, making it suitable for more complex grid patterns.

### **Core Steps**:
1. **Grid Initialization**:  
   The grid is initialized based on the values in the `generate_map.py` file, where each value represents the amount of snow in that cell.
   
2. **Heuristic Selection**:  
   The user can select one of the three heuristics: Vertical, Horizontal, or Manhattan, in the `classes.py` file. This choice affects the pathfinding behavior. Currently the algorithm automatically chooses by itself.

3. **Pathfinding Execution**:  
   The algorithm calculates the most efficient path for the snowplow based on the selected heuristic and begins clearing the snow.

4. **Return to Origin**:  
   If the snowplow’s capacity is exceeded, it returns to the origin (starting point), passing through cleared areas. After unloading, it continues from the last cleared endpoint.

5. **Repeat the Process**:  
   The process continues until all snow is cleared from the grid.

---

## **Future Enhancements**
- Implement dynamic snow accumulation during the clearing process.
- Support for irregularly shaped grids.
- Advanced pathfinding algorithms like **A\*** or **Dijkstra’s Algorithm**.
- Real-time visualization and user interaction.
- Machine learning-based predictive snow distribution for future path planning.

---

## **Contributors**
- [Myat Min Thu](https://github.com/mmthu0121)  
Feel free to contribute by submitting issues or pull requests.

---

## License
This project is licensed under the [MIT License](LICENSE).
