Kieran White, Michael Langevin, Tony Hansen
whit3296@umn.edu, lang1288@umn.edu, hans7638@umn.edu

#####

Step 1: Install Required Packages

- pip install osmnx matplotlib shapely time heapq math random

#####

Step 2: Download code / pull from github
- get the code into a Python file

#####

Step 3: Run the Program
- Navigate to the directory where the file is saved using the terminal or command prompt. Run the program using:

- python pathfinding.py
- (or however you get python files to run. i.e code runner extension)

#####

Step 4: Select a Destination
- Once the program runs, you will see a list of restaurants. Follow the instructions to:

- Select a restaurant by typing the corresponding number and pressing Enter. For example:

1. McDonald's
2. Raising Cane's
...
Enter the number corresponding to the restaurant or 'exit':
If you want to exit the program, type exit and press Enter.

#####

Step 5: Choose Heuristics
- After selecting a restaurant, you will have two options:

Use a single heuristic: Choose one heuristic (e.g., Euclidean Distance, Scenic Heuristic, etc.).
or Run all heuristics: Compare results from all heuristics.
The program will prompt you to choose:

For single heurisitc, enter its corresponding number (1–5).
For all, all heuristics will be ran and results can be found in terminal

#####

Step 6: View Results
The program will compute the shortest path using the selected heuristic(s). It will display:

The path taken, found in seperate pop up tab (ONLY IF TESTING ONE HEURISTIC).
The total distance traveled.
The runtime and number of visited nodes.

#####