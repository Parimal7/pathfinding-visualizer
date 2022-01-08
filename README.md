## Pre-requisites
You need SFML installed. If you are on Ubuntu, simply run `sudo apt-get install libsfml-dev` in your terminal. For other OS, please see how to install SFML from here --> https://www.sfml-dev.org/tutorials/2.5/

## Build Instructions

1. `git clone https://github.com/Parimal7/pathfinding-visualizer.git`
2. `g++ pathfinder.cpp -o pathfinder -std=c++11 -lsfml-window -lsfml-system -lsfml-graphics`
3. `./pathfinder`

## Supported Operations
- You can left click tiles to draw walls on the grid.
- Search using either Dijkstra or A* pathdinging algorithm by clicking the respective buttons.

## TO-DO
1. Add the continuous creation of walls when left mouse button is clicked.
2. Add more search algorithms.
3. Shift to Qt framework from SFML.

## References
1. https://clementmihailescu.github.io/Pathfinding-Visualizer/
2. https://www.sfml-dev.org/tutorials/2.5/
