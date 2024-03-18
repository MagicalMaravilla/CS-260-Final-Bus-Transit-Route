#include <iostream> // For input and output operations, e.g., cout, cin.
#include <vector> // For using the vector container, a dynamic array.
#include <queue> // Provides queue and priority_queue container adaptors.
#include <map> // For using the map container, which stores key-value pairs with unique keys.
#include <limits> // Provides numeric limits for types, useful for setting initial conditions like "infinity".
#include <set> // For using the set container, which stores unique elements in a specific order.
#include <utility> // Includes utility functions, like std::pair and std::swap.
#include <algorithm> // Contains a collection of algorithmic functions, e.g., sort, find.

using namespace std; // Allows direct access to standard library components without the std:: prefix.


/**
 * Represents an undirected graph using an adjacency list where each edge has an associated weight.
 * Supports operations such as adding vertices, adding edges, finding the shortest path (Dijkstra's algorithm),
 * and constructing a Minimum Spanning Tree (MST) using Prim's algorithm.
 */
class Graph {
private:
    // Adjacency list represented by a map from a vertex (char) to a list of pairs (adjacent vertex, weight).
    map<char, vector<pair<char, int>>> adjList;

public:
    /**
     * Checks if the graph is empty.
     * @return True if the graph has no vertices, false otherwise.
     */
    bool is_empty() const {
        return adjList.empty();
    }

    /**
     * Adds a new vertex to the graph if it does not already exist.
     * @param vertex_name The name (label) of the vertex to add.
     */
    void add_vertex(char vertex_name) {
        if (adjList.find(vertex_name) == adjList.end()) {
            adjList[vertex_name] = {};
            cout << "Vertex " << vertex_name << " added." << endl;
        } else {
            cout << "Vertex " << vertex_name << " already exists." << endl;
        }
    }

    /**
     * Adds an edge to the graph between two vertices with the specified weight.
     * The graph is undirected, so the edge is added in both directions.
     * Validates that both vertices exist and the weight is positive before adding the edge.
     * @param source The source vertex.
     * @param destination The destination vertex.
     * @param weight The weight of the edge.
     */
    void add_edge(char source, char destination, int weight) {
        if (weight <= 0) {
            cout << "Error: Edge weight must be positive." << endl;
            return;
        }
        if (adjList.find(source) == adjList.end() || adjList.find(destination) == adjList.end()) {
            cout << "Error: One or both vertices do not exist." << endl;
            return;
        }
        adjList[source].emplace_back(destination, weight);
        adjList[destination].emplace_back(source, weight);
    }

    /**
     * Finds the shortest path between two vertices using Dijkstra's algorithm.
     * @param source The starting vertex.
     * @param destination The target vertex.
     * @return A vector of characters representing the shortest path, including both source and destination.
     */
    vector<char> shortest_path(char source, char destination) const {
        if (is_empty() || adjList.find(source) == adjList.end() || adjList.find(destination) == adjList.end()) {
            cout << "Error: Source or destination vertex does not exist or graph is empty." << endl;
            return {};
        }

        map<char, int> distances;
        map<char, char> predecessors;
        auto comp = [](const pair<int, char>& left, const pair<int, char>& right) { return left.first > right.first; };
        priority_queue<pair<int, char>, vector<pair<int, char>>, decltype(comp)> pq(comp);

        for (const auto& vertex : adjList) {
            distances[vertex.first] = numeric_limits<int>::max();
        }
        distances[source] = 0;
        pq.emplace(0, source);

        set<char> visited;

        while (!pq.empty()) {
            char current_vertex = pq.top().second;
            pq.pop();

            if (!visited.insert(current_vertex).second) continue;
            if (current_vertex == destination) break;

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

        vector<char> path;
        for (char at = destination; at != source; at = predecessors.at(at)) {
            path.push_back(at);
        }
        path.push_back(source);
        reverse(path.begin(), path.end());
        return path;
    }

    /**
     * Constructs the Minimum Spanning Tree (MST) of the graph using Prim's algorithm.
     * @return A map representing the MST in the same structure as the adjacency list.
     */
    map<char, vector<pair<char, int>>> min_span_tree() const {
        if (is_empty()) {
            cout << "Error: Graph is empty. MST cannot be created." << endl;
            return {};
        }

        set<char> in_mst;
        map<char, vector<pair<char, int>>> mst;
        auto comp = [](const pair<int, pair<char, char>>& left, const pair<int, pair<char, char>>& right) { return left.first > right.first; };
        priority_queue<pair<int, pair<char, char>>, vector<pair<int, pair<char, char>>>, decltype(comp)> edges(comp);

        char start = adjList.begin()->first;
        in_mst.insert(start);
        for (const auto& edge : adjList.at(start)) {
            edges.emplace(edge.second, make_pair(start, edge.first));
        }

        while (!edges.empty()) {
            auto [weight, vertices] = edges.top();
            edges.pop();
            char from = vertices.first;
            char to = vertices.second;

            if (in_mst.insert(to).second) {
                mst[from].emplace_back(to, weight);
                for (const auto& edge : adjList.at(to)) {
                    if (in_mst.find(edge.first) == in_mst.end()) {
                        edges.emplace(edge.second, make_pair(to, edge.first));
                    }
                }
            }
        }

        if (mst.size() != adjList.size()) {
            cout << "The graph is disconnected. MST cannot be created for the entire graph." << endl;
            return {};
        }

        // Print MST and calculate total weight
        cout << "Minimum Spanning Tree (MST):" << endl;
        int totalWeight = 0;
        for (const auto& [from, edges] : mst) {
            for (const auto& [to, weight] : edges) {
                cout << from << " - " << to << " : " << weight << endl;
                totalWeight += weight;
            }
        }
        cout << "Total weight (miles) of the MST: " << totalWeight << endl;
        return mst;
    }
};

int main() {
    // Implementation of tests and demonstration of Graph class functionalities.
    Graph g;

    // Adding vertices
    cout << "Adding vertices A through J." << endl;
    for (char vertex = 'A'; vertex <= 'J'; ++vertex) {
        g.add_vertex(vertex);
    }

    // Defining edges based on bus routes
    cout << "\nDefining bus routes as edges with distances." << endl;
    vector<tuple<char, char, int>> routes = {
        {'A', 'B', 50}, {'A', 'D', 150}, {'A', 'E', 200}, {'B', 'C', 40},
        {'B', 'E', 60}, {'C', 'G', 120}, {'D', 'E', 90}, {'E', 'F', 80},
        {'F', 'G', 70}, {'G', 'H', 100}, {'G', 'I', 60}, {'H', 'I', 80},
        {'H', 'J', 140}, {'I', 'J', 120}
    };
    for (const auto& [start, end, distance] : routes) {
        g.add_edge(start, end, distance);
    }

    // Invalid inputs tests
    cout << "\nTesting response to invalid inputs:" << endl;
    g.add_edge('A', 'Z', 10);  // Non-existent vertex
    g.add_edge('A', 'B', -5);  // Invalid weight

    // Shortest paths tests
    cout << "\nTesting shortest path calculations:" << endl;
    auto printPath = [](const string& description, const vector<char>& path) {
        cout << description << ": ";
        for (char p : path) cout << p << " ";
        cout << endl;
    };

    printPath("Shortest path from A to B", g.shortest_path('A', 'B'));
    printPath("Shortest path from A to F", g.shortest_path('A', 'F'));
    printPath("Shortest path from A to J", g.shortest_path('A', 'J'));
    printPath("Shortest path from A to Z (expected failure)", g.shortest_path('A', 'Z'));

    // Extended testing for paths back to home bases A and J from other vertices
cout << "\nTesting shortest paths to home bases A and J:" << endl;
vector<char> vertices = {'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};
for (char vertex : vertices) {
    // Testing paths to home base A
    string descriptionToA = "Shortest path from " + string(1, vertex) + " to A";
    vector<char> pathToA = g.shortest_path(vertex, 'A');
    printPath(descriptionToA, pathToA);

    // Testing paths to home base J
    string descriptionToJ = "Shortest path from " + string(1, vertex) + " to J";
    vector<char> pathToJ = g.shortest_path(vertex, 'J');
    printPath(descriptionToJ, pathToJ);
}

     cout << "\nConstructing and testing the Minimum Spanning Tree:" << endl;
    auto mst = g.min_span_tree(); // Assuming min_span_tree returns the MST

    // Calculate and print the total weight of the MST
    int totalWeight = 0;
    for (const auto& [from, edges] : mst) {
        for (const auto& [to, weight] : edges) {
            totalWeight += weight; // Accumulate the weight
        }
    }
    cout << "Total weight (miles) of the MST: " << totalWeight << endl;

    return 0;
}
