import osmnx as ox
import matplotlib.pyplot as plt  
from shapely.geometry import box
import time
import heapq
import math
import random

# First we will need to set the bounds of the map #
#we picked 4 coordinates that will be used to set the top, bottom, and side bounds of the map of campus that will have all of the restuarants we will be pathfinding to
north, south, east, west = 44.98795, 44.96187, -93.25482, -93.21358
# These are restaurant nodes that will be represented as large red nodes on the map of campus
#all of our coordinate pairs were sourced using a third party maping app
red_nodes_coords = {
    "McDonald's": (44.98032, -93.23443),
    "Raising Cane's": (44.97905, -93.23485),
    "Blue Door": (44.98772, -93.23002),
    "Domino's": (44.97419, -93.22676),
    "Chick-fil-A": (44.97404, -93.22921),
    "Pho Mai": (44.98017, -93.23638),
    "Frank & Andrea": (44.98130, -93.23769),
    "Blaze Pizza": (44.97347, -93.22340),
    "Buffalo Wild Wings": (44.97600, -93.22673),
    "The Corner Bar": (44.97283, -93.24766),
    "Maxwell's Cafe and Grill": (44.98385, -93.24377),
    "Alma": (44.98385, -93.24805),
    "Potter's Pasties & Pies": (44.98762, -93.22586),
    "Fred's Chicken 'N' Waffles": (44.96904, -93.24523),
    "Davanni's Pizza and Hot Hoagies": (44.96562, -93.23593),
}
# Define the coordinates for our starting node
# we will always be starting the pathfinding algortihm from coffman memorial union
starting_node_coords = (44.97303, -93.23532)

#these are scenic locations for the scenic heuristic we are doing
scenic_locations = [
     (44.977034, -93.227070),  # Huntington Bank Stadium
     (44.973194471442845, -93.23705506708603),  # Weisman Art Museum
     (44.976244845144564, -93.2353631884106),   # Northrup Auditorium
     (44.975112, -93.240431), #bohemian flats riverside park
     (44.982340, -93.236528) #dinkytown main street
]

#Define heuristic functions

#euclidiean distance-direct path from start to selected restaurant that acts as a bird flying directly to the location to guide the Astar's decisions
def heuristic_euclidean(u, v, graph):
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    
    return ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5
#Manhattan Distance is finds the distance between coffman and the selected location based on horizontal and vertical movement to guide A*
def heuristic_manhattan(u, v, graph):
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    
    return abs(u_coords[0] - v_coords[0]) + abs(u_coords[1] - v_coords[1])
#We want to see how having no hueristic effects our runtime and path finding
def heuristic_zero(u, v, graph):
    return 0  # Acts as Dijkstra's algorithm
#this is a scenic pathfinding heuristic that gives heurisitc weight towards A* passing by the scenic locations we instantiated above
def heuristic_scenic(u, v, graph):
    #declare graphing coords of the startign point and desitination that the user picks
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    #this will use the euclidean distance as a foundation
    direct_distance = ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5
    #but then weight each decision with a scenic weight
    scenic_weight = 0
    #this weight is based on the euclidian distance of the algorithm the node is looking at, to the list of scenic locations
    for scenic_loc in scenic_locations:
        scenic_dist = ((u_coords[0] - scenic_loc[0]) ** 2 + (u_coords[1] - scenic_loc[1]) ** 2) ** 0.5
        scenic_weight += max(0, 1.0 / (scenic_dist * 0.001))
    scenic_bias = -5000
    total_heuristic = direct_distance + scenic_bias * scenic_weight
    
    return total_heuristic
#This eval function will choose a random factor to subtract from the direct distance to see how it effects the path
def heuristic_random(u, v, graph):
    # initialize the graph nodes of coffman and the destination
    u_coords = (graph.nodes[u]['y'], graph.nodes[u]['x'])
    v_coords = (graph.nodes[v]['y'], graph.nodes[v]['x'])
    #we use the euclidean distance as a foundation for this heuristic as well 
    direct_distance = ((u_coords[0] - v_coords[0]) ** 2 + (u_coords[1] - v_coords[1]) ** 2) ** 0.5
    #this will generate a random factor between 0 and 1000000
    random_factor = random.uniform(0, 1000000)
    #now we will weight the euclidean heuristic with the random factor and return that evaluation
    total_heuristic = direct_distance - random_factor
    
    return total_heuristic


#The Core of our code is this A* function that finds the paths through the map using whatever heuristics the user selects
def Astar(graph, start, goal, heuristic):
    #Priority queue for nodes
    open_set = []  
    #push the start node to the queue
    heapq.heappush(open_set, (0, start))  
    #visited nodes will be saved on here to retrace the path
    came_from = {}  
    #Distance from start to each node
    g_score = {node: float('inf') for node in graph.nodes}  
    g_score[start] = 0
    #approximate distance from start to goal
    f_score = {node: float('inf') for node in graph.nodes}
    f_score[start] = heuristic(start, goal, graph)
    #this will count the number of visited nodes
    visited_nodes_count = 0
    #this will keep looping while there are nodes in the open set
    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        #if the node that is being looked at is the goal, then it will append all of the visited nodes to the path
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            #Returns the reversed path and the count of visited nodes
            return list(reversed(path)), visited_nodes_count 
        #this for loop calculates the cost to reach each neighbor 
        for neighbor in graph.neighbors(current_node):
            tentative_g_score = g_score[current_node] + graph[current_node][neighbor][0]['length']
            #compares calulated paths and updates current node
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal, graph)
                
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        visited_nodes_count += 1
    #for if no path is found
    return float('inf') 

#these will be a library of how the user can access the 
heuristics = {
    "1": ("Euclidean Distance", heuristic_euclidean),
    "2": ("Manhattan Distance", heuristic_manhattan),
    "3": ("No Heuristic (Dijkstra)", heuristic_zero),
    "4": ("Scenic Route Heuristic", heuristic_scenic),
    "5": ("heuristic_random", heuristic_random)
}

#execute code
try:
    #these will create the map and instantiate the nodes and edges
    bbox_polygon = box(west, south, east, north)
    graph = ox.graph_from_polygon(bbox_polygon, network_type="walk")

    if graph.number_of_edges() == 0:
        raise ValueError("The graph contains no edges. Try a different area or network type.")

    starting_node = ox.nearest_nodes(graph, X=starting_node_coords[1], Y=starting_node_coords[0])

    scenic_nodes = []
    for coords in scenic_locations:
        node = ox.nearest_nodes(graph, X=coords[1], Y=coords[0])
        scenic_nodes.append(node)

    while True:
        #Asks the user to select a restaurant
        print("\nSelect a restaurant to find the shortest path to, or type 'exit' to quit:")
        for i, restaurant in enumerate(red_nodes_coords.keys(), start=1):
            print(f"{i}. {restaurant}")

        user_input_restaurant = input("Enter the number corresponding to the restaurant or 'exit': ").strip().lower()

        if user_input_restaurant == "exit":
            print("Exiting the program. Goodbye!")
            break

        if not user_input_restaurant.isdigit() or int(user_input_restaurant) < 1 or int(user_input_restaurant) > len(red_nodes_coords):
            print("Invalid input. Please enter a valid number or 'exit'.")
            continue

        selected_index = int(user_input_restaurant)
        restaurant_names = list(red_nodes_coords.keys())
        selected_restaurant = restaurant_names[selected_index - 1]
        destination_coords = red_nodes_coords[selected_restaurant]
        GOAL = destination_coords = red_nodes_coords[selected_restaurant]

        #asks the user if they want to use all heuristics
        print("\nSelect an option:")
        print("1. Use a single heuristic")
        print("2. Use all three heuristics and compare results")

        user_input_option = input("Enter the number corresponding to the option: ").strip()

        if user_input_option == "1":
            #Asks the user to select a heuristic
            print("\nSelect a heuristic for the A* search:")
            for key, (name, _) in heuristics.items():
                print(f"{key}. {name}")
            user_input_heuristic = input("Enter the number corresponding to the heuristic: ").strip()
            if user_input_heuristic not in heuristics:
                print("Invalid heuristic selection. Please try again.")
                continue
            heuristic_name, heuristic_function = heuristics[user_input_heuristic]
            #This will find the nearest node to the selected desitination
            destination_node = ox.nearest_nodes(graph, X=destination_coords[1], Y=destination_coords[0])
            #starts measuring the time it takes to run
            start_time = time.time()
            #this will try to find the shortest path from the starting node to the destination node using our A*
            try:
                path, visited_nodes = Astar(graph, starting_node, destination_node, heuristic=heuristic_function)

                #this stops the clock we set earlier 
                end_time = time.time()
                #we can calculate the time it took by doing this
                time_taken = end_time - start_time
                print(f"\nShortest path to {selected_restaurant} using {heuristic_name} visualized successfully.")
                print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")
                print(f"Total number of nodes visited: {visited_nodes}")

                # Generate the map when using a single heuristic
                node_color = ['lightgreen' if node == starting_node else ('r' if node == destination_node else ('purple' if node in scenic_nodes else 'gray'))for node in graph.nodes]
                node_size = [35 if node == starting_node or node == destination_node or node in scenic_nodes else 0 for node in graph.nodes]
                path_edges = list(zip(path[:-1], path[1:]))
                edge_color = ['b' if (u, v) in path_edges or (v, u) in path_edges else 'w' for u, v, k in graph.edges]
                
                

                #Plots the graph
                fig, ax = ox.plot_graph(
                    graph,
                    bgcolor='k',
                    node_color=node_color,
                    edge_color=edge_color,
                    node_size=node_size,
                    edge_linewidth=[3 if (u, v) in path_edges or (v, u) in path_edges else 0.5 for u, v, k in graph.edges],
                    figsize=(20, 16),
                    dpi=100,
                    show=False  
                )

                plt.show(block=False) 
                plt.pause(0.001)       

                print("\n")
                print("-----------------------------------------------------------------------------------------")
                print("-----------------------------------------------------------------------------------------")
                print("\n")

            except Exception as ex:
                print(f"An error occurred during pathfinding: {ex}")

        elif user_input_option == "2":
            # this will compare the results
            print("\nComparing results using all three heuristics:")

            destination_node = ox.nearest_nodes(graph, X=destination_coords[1], Y=destination_coords[0])

            for key, (heuristic_name, heuristic_function) in heuristics.items():
                #start the clock
                start_time = time.time()

                #find the shortest path via the A* algo
                try:
                    path, visited_nodes = Astar(graph, starting_node, destination_node, heuristic=heuristic_function)

                    #stop the clock
                    end_time = time.time()
                    #calculate the runtime
                    time_taken = end_time - start_time
                    #THis will print the results for each heuristic. 
                    print(f"\n{heuristic_name} Heuristic:")
                    print(f"Time taken for pathfinding: {time_taken:.8f} seconds.")
                    print(f"Total number of nodes visited: {visited_nodes}")

                except Exception as ex:
                    print(f"An error occurred during pathfinding with {heuristic_name} heuristic: {ex}")

            print("\nComparison complete.")
            print("\n-----------------------------------------------------------------------------------------")
            print("-----------------------------------------------------------------------------------------")
            print("\n")

        else:
            print("Invalid option. Please try again.")

except Exception as e:
    print(f"An error occurred: {e}")
