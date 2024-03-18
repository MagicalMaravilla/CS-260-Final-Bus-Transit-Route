#include "graph.h" // Includes the Graph class definition.
#include <iostream> // Used for input and output operations.
#include <limits> // Provides access to numeric limits (e.g., for maximum integer values).
#include <algorithm> // Includes various algorithm functions, such as sort.
#include <vector> // For using dynamic arrays that can change in size.
#include <queue> // Includes queue for FIFO operations and priority_queue for sorted access.
#include <map> // For using maps, which store key-value pairs sorted by keys.
#include <utility> // Contains utility functions, like pair for storing two values as a single object.

// Default constructor: Initializes a new Graph object.
Graph::Graph() {
    // No explicit actions needed; the adjacency list is automatically initialized by the map's constructor.
}

// Checks if the graph contains no vertices.
bool Graph::is_empty() const {
    // Returns true if the adjacency list is empty, indicating no vertices are present.
    return adjList.empty();
}

// Adds a new vertex to the graph.
void Graph::add_vertex(char vertex_name) {
    // Checks if the vertex already exists to avoid duplication.
    if (adjList.find(vertex_name) == adjList.end()) {
        // If not found, adds the vertex with an empty adjacency list.
        adjList[vertex_name] = {};
        std::cout << "Vertex " << vertex_name << " added." << std::endl;
    } else {
        // If the vertex already exists, notifies the user.
        std::cout << "Vertex " << vertex_name << " already exists." << std::endl;
    }
}

// Adds an undirected edge between two vertices.
void Graph::add_edge(char source, char destination, int weight) {
    // Validates the edge weight to be positive.
    if (weight <= 0) {
        std::cout << "Error: Edge weight must be positive." << std::endl;
        return;
    }
    // Checks the existence of both vertices before adding an edge.
    if (adjList.find(source) == adjList.end() || adjList.find(destination) == adjList.end()) {
        std::cout << "Error: One or both vertices do not exist." << std::endl;
        return;
    }
    // Adds the edge in both directions (undirected graph) with the given weight.
    adjList[source].emplace_back(destination, weight);
    adjList[destination].emplace_back(source, weight);
}

// Finds the shortest path between two vertices using Dijkstra's algorithm.
std::vector<char> Graph::shortest_path(char source, char destination) const {
    // Implementation for Dijkstra's algorithm, calculating shortest paths from the source to all vertices.
    // Finally returns the shortest path from source to destination.

    // Checks for the existence of both source and destination vertices.
    if (is_empty() || adjList.find(source) == adjList.end() || adjList.find(destination) == adjList.end()) {
        std::cout << "Error: Source or destination vertex does not exist or graph is empty." << std::endl;
        return {};
    }

    // Distance map to hold the shortest distance from source to each vertex.
    std::map<char, int> distances;
    // Predecessor map to reconstruct the path from source to destination.
    std::map<char, char> predecessors;
    // Priority queue to select the vertex with the minimum distance next.
    auto comp = [](const std::pair<int, char>& left, const std::pair<int, char>& right) { return left.first > right.first; };
    std::priority_queue<std::pair<int, char>, std::vector<std::pair<int, char>>, decltype(comp)> pq(comp);

    // Initializes distances as infinite for all vertices except the source.
    for (const auto& vertex : adjList) {
        distances[vertex.first] = std::numeric_limits<int>::max();
    }
    distances[source] = 0; // Distance from source to itself is 0.
    pq.emplace(0, source); // Starts with the source vertex.

    std::set<char> visited; // Tracks visited vertices to avoid re-processing.

    while (!pq.empty()) {
        // Extracts vertex with minimum distance.
        char current_vertex = pq.top().second;
        pq.pop();

        // Continues if this vertex has already been processed.
        if (!visited.insert(current_vertex).second) continue;
        // Breaks the loop if the destination vertex is reached.
        if (current_vertex == destination) break;

        // Relaxation step for all adjacent vertices.
        for (const auto& edge : adjList.at(current_vertex)) {
            char adj_vertex = edge.first;
            int weight = edge.second;

            if (distances[adj_vertex] > distances[current_vertex] + weight) {
                distances[adj_vertex] = distances[current_vertex] + weight;
                predecessors[adj_vertex] = current_vertex;
                pq.emplace(distances[adj_vertex], adj_vertex);
            }
        }
    }

    // Constructs the shortest path using the predecessor map.
    std::vector<char> path;
    // Handles the case where no path exists.
    if (predecessors.find(destination) == predecessors.end()) {
        return {}; // No path found.
    }
    // Reconstructs the path from destination to source.
    for (char at = destination; at != source; at = predecessors.at(at)) {
        path.push_back(at);
    }
    path.push_back(source);
    std::reverse(path.begin(), path.end()); // Reverses the path to start from the source.
    return path; // Returns the shortest path.
}

// Constructs the Minimum Spanning Tree (MST) of the graph using Prim's algorithm.
std::map<char, std::vector<std::pair<char, int>>> Graph::min_span_tree() const {
    // Checks if the graph is empty.
    if (is_empty()) {
        std::cout << "Error: Graph is empty. MST cannot be created." << std::endl;
        return {};
    }

    // Initializes the MST construction process.
    std::set<char> in_mst; // Tracks vertices included in the MST.
    std::map<char, std::vector<std::pair<char, int>>> mst; // Stores the MST as an adjacency list.
    auto comp = [](const std::pair<int, std::pair<char, char>>& left, const std::pair<int, std::pair<char, char>>& right) { return left.first > right.first; };
    std::priority_queue<std::pair<int, std::pair<char, char>>, std::vector<std::pair<int, std::pair<char, char>>>, decltype(comp)> edges(comp);

    // Starts with the first vertex and explores all connecting edges.
    char start = adjList.begin()->first;
    in_mst.insert(start);
    for (const auto& edge : adjList.at(start)) {
        edges.emplace(edge.second, std::make_pair(start, edge.first));
    }

    // Continues adding the lightest edge that connects the MST with a new vertex.
    while (!edges.empty()) {
        auto [weight, vertices] = edges.top();
        edges.pop();
        char from = vertices.first;
        char to = vertices.second;

        // Adds the edge to the MST if it connects to a new vertex.
        if (in_mst.insert(to).second) {
            mst[from].emplace_back(to, weight);
            // Explores new connections from the added vertex.
            for (const auto& edge : adjList.at(to)) {
                if (in_mst.find(edge.first) == in_mst.end()) {
                    edges.emplace(edge.second, std::make_pair(to, edge.first));
                }
            }
        }
    }

    // Checks if all vertices are included in the MST.
    if (mst.size() != adjList.size()) {
        std::cout << "The graph is disconnected. MST cannot be created for the entire graph." << std::endl;
        return {};
    }

    return mst; // Returns the constructed MST.
}
