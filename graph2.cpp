// Include the graph class definition from "graph2.h".
#include "graph2.h"

// Include for input/output operations, such as printing to the console.
#include <iostream>

// Include for using numeric limits, which can be useful for setting initial distances in algorithms like Dijkstra's.
#include <limits>

// Include for using algorithms such as std::sort.
#include <algorithm>

// Include for using dynamic array containers that can grow or shrink in size.
#include <vector>

// Include for using queue and priority_queue, useful for implementing BFS and priority-based selections like in Prim's algorithm.
#include <queue>

// Include for using associative containers that store elements formed by the combination of a key value and a mapped value.
#include <map>

// Include for using utility functions, such as std::pair, which can represent edges in a graph (pairing vertices with weights).
#include <utility>


// Constructor: Initializes a Graph object with an empty adjacency list.
Graph::Graph() {
    // The adjacency list (adjList) is automatically initialized by the map's default constructor.
}

// Checks if the graph contains no vertices.
bool Graph::is_empty() const {
    return adjList.empty(); // Returns true if the adjacency list is empty.
}

// Adds a new vertex to the graph if it doesn't already exist.
void Graph::add_vertex(char vertex_name) {
    // Checks if the vertex already exists in the graph.
    if (adjList.find(vertex_name) == adjList.end()) {
        adjList[vertex_name] = {}; // Adds the vertex with an empty adjacency list.
        std::cout << "Vertex " << vertex_name << " added." << std::endl;
    } else {
        std::cout << "Vertex " << vertex_name << " already exists." << std::endl;
    }
}

// Adds an edge between two vertices with a specified weight.
void Graph::add_edge(char source, char destination, int weight) {
    // Validates the weight of the edge; it must be positive.
    if (weight <= 0) {
        std::cout << "Error: Edge weight must be positive." << std::endl;
        return;
    }
    // Checks if both vertices exist. If not, prints an error message.
    if (adjList.find(source) == adjList.end() || adjList.find(destination) == adjList.end()) {
        std::cout << "Error: One or both vertices do not exist." << std::endl;
        return;
    }
    // Adds the edge in both directions to the adjacency list, signifying an undirected graph.
    adjList[source].emplace_back(destination, weight);
    adjList[destination].emplace_back(source, weight);
}

// Implements Dijkstra's algorithm to find the shortest path from a source to a destination vertex.
std::vector<char> Graph::shortest_path(char source, char destination) const {
    // Implementation specifics for finding the shortest path.
}

// Constructs the Minimum Spanning Tree (MST) using Prim's algorithm.
std::map<char, std::vector<std::pair<char, int>>> Graph::min_span_tree() const {
    // Checks if the graph is empty before attempting to create an MST.
    if (is_empty()) {
        std::cout << "Error: Graph is empty. MST cannot be created." << std::endl;
        return {};
    }

    // Prepares for MST construction.
    std::set<char> inMST; // Tracks vertices included in MST.
    std::map<char, std::vector<std::pair<char, int>>> mst; // Stores the MST.
    std::priority_queue<std::pair<int, char>, std::vector<std::pair<int, char>>, std::greater<>> pq; // Min heap for edges.
    std::map<char, int> key; // Stores the minimum weight to connect a vertex to the MST.
    std::map<char, char> parent; // Tracks the parent vertex in the MST for each vertex.

    // Initializes keys as infinite and parents as undefined.
    for (const auto& vertex : adjList) {
        key[vertex.first] = std::numeric_limits<int>::max();
        parent[vertex.first] = -1;
    }

    // Starts MST construction from the first vertex.
    char start = adjList.begin()->first;
    pq.push({0, start}); // Starts with the first vertex, with a weight of 0 to itself.
    key[start] = 0;

    // Constructs the MST.
    while (!pq.empty()) {
        // Extracts the vertex with the minimum key.
        char u = pq.top().second;
        pq.pop();

        // Prevents re-including a vertex in MST.
        if (!inMST.insert(u).second) continue;

        // Updates the adjacent vertices of the extracted vertex.
        for (const auto& [v, weight] : adjList.at(u)) {
            // If v is not in MST and its key is greater than the new weight.
            if (inMST.find(v) == inMST.end() && weight < key[v]) {
                key[v] = weight; // Updates the key.
                pq.push({key[v], v}); // Adds the vertex to the priority queue.
                parent[v] = u; // Sets u as the parent of v in MST.
            }
        }
    }

    // Assembles the resulting MST from the parent map.
    for (const auto& [vertex, _] : adjList) {
        if (parent[vertex] != -1) { // Ensures the vertex is not the start vertex.
            mst[parent[vertex]].push_back({vertex, key[vertex]});
        }
    }

    // Checks if all vertices are included in the MST.
    return mst;
}
