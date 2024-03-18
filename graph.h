// Ensures the header is only included once during compilation
#pragma once

// Used for storing lists of adjacent vertices and their weights
#include <vector> 

// Used for implementing algorithms like Dijkstra's
#include <queue>  

// Maps each vertex to its list of adjacent vertices (adjacency list)
#include <map>    

// Used to track vertices visited in algorithms
#include <set>    

// Represents an undirected graph using an adjacency list.
class Graph {
public:
    Graph(); // Initializes a new, empty Graph

    // Returns true if the graph has no vertices
    bool is_empty() const;

    // Adds a new vertex with the given name to the graph if it's not already present
    void add_vertex(char vertex_name);

    // Connects two vertices with an undirected edge of the given weight
    void add_edge(char source, char destination, int weight);

    // Computes the shortest path between two vertices using Dijkstra's algorithm
    std::vector<char> shortest_path(char source, char destination) const;

    // Constructs and returns the Minimum Spanning Tree (MST) of the graph using Prim's algorithm
    std::map<char, std::vector<std::pair<char, int>>> min_span_tree() const;

private:
    // Maps each vertex to a vector of pairs, where each pair is an adjacent vertex and the weight of the edge connecting them
    std::map<char, std::vector<std::pair<char, int>>> adjList;
};
