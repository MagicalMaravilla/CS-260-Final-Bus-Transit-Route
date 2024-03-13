# CS-260-Final-Bus-Transit-Route
I will make a bus route that allows me to find the most efficient pathway to each location based on different locations on the map

General Design:
We could create a transportation netowrk, where the locations are represented as verticies we can do A - J (to give enough of an example) and the pathways between them are the edges with associated travel distences (weights). 
The goal is to find the most efficent routes between locations and to construct a small network that connects all locations to the least total distances.

Graph reprensentation:
Verticies (Locations): Each vertex represents a location, we could use a 'char' for the vertex id, allowing easy mapping of named locations winith a manageble size graph.
Edges(Paths): Edges represent direct routes between locations. These are undirected, meaning that travel is possible in both directions with the same distance.
Weights(Distances): Weights on the edges show the distance or "cost" associated with the traveling that pathways

Solving probems with the graph:

Shortest Path (Dijkstra's Algo)
Problem: Finding shortest path between two locations.
Solution: Demonstrate with Dijkstra's algorithm how it efficently finds the shortest path from a starting location to a destination cosidering the total distances. 
This will be really helpful for route planning and transportation network, making sure the least distance is traveled between points.

Minimum Spanning Tree (Prim's Algo)
Problem: Constructing a small network connecting all locations with the least total distance
Solution: Prim's algorithm finds a Minimum Spanning Tree (MST) that spans all verticies with the minimum possible total edge weight. 
This could represent the least costly way to build pathways that connect all locations in a new or existing network

Implementation General Overview:

Graph Class: Central to the design, summerizing the graph structure and algorithims.
add_vertex and add edge: build the graph by adding locations and their connecting routes.
shortest_path: Solve for the shorest route between two locations
min_span_tree: Generate a minimal connecting network of all locations.

Usage Scenario

Transportation Network Planning: The graph could create a structure and its algorithms could be used by me to create the start of a logigistic layout. 
which will optimize routes, reduce travel times, and have efficent transportation networks.

Final Thoughts:

The design in mind for using a graph structue problem solves mapping elements to graph with verts edges and wegiths. 
My Algorithim choices are the backbone to my general ability to create a functional transportation netowrk and solving for opimization, based on the current needs ive labeled above

This is what I have thought of so far, i may implement more if possible when doing my research.
