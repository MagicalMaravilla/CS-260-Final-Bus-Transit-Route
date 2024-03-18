#include "graph.h" // Include the Graph class definition.
#include <iostream> // For input and output operations.
#include <vector>   // To use the vector container for storing routes.
#include <tuple>    // To store each route as a tuple (start vertex, end vertex, weight).
#include <string>   // For string operations (not explicitly used in this snippet).

using namespace std;

int main() {
    Graph g; // Initialize an instance of the Graph class.

    // Notify the user that vertices are being added.
    cout << "Adding vertices A through J." << endl;
    for (char vertex = 'A'; vertex <= 'J'; ++vertex) {
        g.add_vertex(vertex); // Add vertices labeled 'A' to 'J' to the graph.
    }

    // Define the bus routes (edges) with specified distances (weights) between vertices.
    cout << "\nDefining bus routes as edges with distances." << endl;
    vector<tuple<char, char, int>> routes = {
        // Each tuple represents a direct route between two vertices and its distance.
        {'A', 'B', 50}, {'A', 'D', 150}, {'A', 'E', 200}, {'B', 'C', 40},
        {'B', 'E', 60}, {'C', 'G', 120}, {'D', 'E', 90}, {'E', 'F', 80},
        {'F', 'G', 70}, {'G', 'H', 100}, {'G', 'I', 60}, {'H', 'I', 80},
        {'H', 'J', 140}, {'I', 'J', 120}
    };
    for (const auto& [start, end, distance] : routes) {
        g.add_edge(start, end, distance); // Add each route to the graph as an edge.
    }

    // Check how the graph handles invalid inputs for edge creation.
    cout << "\nTesting response to invalid inputs:" << endl;
    g.add_edge('A', 'Z', 10);  // Attempt to add an edge to a non-existent vertex 'Z'.
    g.add_edge('A', 'B', -5);  // Attempt to add an edge with an invalid negative weight.

    // Demonstrate shortest path calculations between various vertices.
    cout << "\nTesting shortest path calculations:" << endl;
    auto printPath = [](const string& description, const vector<char>& path) {
        cout << description << ": ";
        for (char p : path) cout << p << " "; // Print each vertex in the path.
        cout << endl;
    };

    // Calculate and print the shortest paths for given pairs of vertices.
    printPath("Shortest path from A to B", g.shortest_path('A', 'B'));
    printPath("Shortest path from A to F", g.shortest_path('A', 'F'));
    printPath("Shortest path from A to J", g.shortest_path('A', 'J'));
    // A path expected to fail due to the non-existent destination 'Z'.
    printPath("Shortest path from A to Z (expected failure)", g.shortest_path('A', 'Z'));

    // Additional tests for shortest paths from various vertices back to 'A' and 'J'.
    cout << "\nTesting shortest paths to home bases A and J:" << endl;
    vector<char> vertices = {'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'};
    for (char vertex : vertices) {
        // For each vertex, compute and print the shortest path back to 'A' and 'J'.
        string descriptionToA = "Shortest path from " + string(1, vertex) + " to A";
        vector<char> pathToA = g.shortest_path(vertex, 'A');
        printPath(descriptionToA, pathToA);

        string descriptionToJ = "Shortest path from " + string(1, vertex) + " to J";
        vector<char> pathToJ = g.shortest_path(vertex, 'J');
        printPath(descriptionToJ, pathToJ);
    }

    // Construct and test the Minimum Spanning Tree (MST) for the graph.
    cout << "\nConstructing and testing the Minimum Spanning Tree:" << endl;
    auto mst = g.min_span_tree(); // Generate the MST using Prim's algorithm.

    // Calculate and display the total weight of the generated MST.
    int totalWeight = 0;
    for (const auto& [from, edges] : mst) {
        for (const auto& [to, weight] : edges) {
            totalWeight += weight; // Sum the weights of all edges in the MST.
            cout << from << " - " << to << " : " << weight << endl; // Print each edge in the MST.
        }
    }
    cout << "Total weight (miles) of the MST: " << totalWeight << endl; // Print the total weight of the MST.

    return 0; // End of the main function.
}
