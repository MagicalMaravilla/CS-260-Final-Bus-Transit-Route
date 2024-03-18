// Prevents the header file from being included multiple times in the same file, avoiding redefinition errors.
#pragma once

// Includes the vector library to use dynamic arrays that can change in size.
#include <vector>
// Includes the queue library for using FIFO (First-In, First-Out) data structures and priority queues.
#include <queue>
// Includes the map library for creating a collection of key-value pairs, sorted by keys, to represent adjacency lists.
#include <map>
// Includes the set library to maintain a collection of unique items, useful for tracking visited vertices.
#include <set>

// Represents an undirected graph using an adjacency list.
class Graph {
public:
    Graph(); // Constructor declaration.
    bool is_empty() const; // Checks if the graph is empty.
    void add_vertex(char vertex_name); // Adds a vertex to the graph.
    void add_edge(char source, char destination, int weight); // Adds an edge to the graph.
    std::vector<char> shortest_path(char source, char destination) const; // Finds the shortest path using Dijkstra's algorithm.
    std::map<char, std::vector<std::pair<char, int>>> min_span_tree() const; // Constructs the MST using Prim's algorithm.

private:
    std::map<char, std::vector<std::pair<char, int>>> adjList; // Adjacency list representation of the graph.
};
