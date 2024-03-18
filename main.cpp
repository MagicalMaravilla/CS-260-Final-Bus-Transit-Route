// Include the header file for the Graph class.
#include "graph2.h"
// For input and output operations.
#include <iostream>
// To use the vector container for storing routes. 
#include <vector>
// To store each route as a tuple of start vertex, end vertex, and weight.   
#include <tuple> 
// Included for string operations.
#include <string>   

using namespace std;

int main() {
    Graph g; // Create an instance of the Graph class.

    // Notify the user that vertices are being added.
    cout << "Adding vertices A through J." << endl;
    // Loop to add vertices 'A' to 'J' to the graph.
    for (char vertex = 'A'; vertex <= 'J'; ++vertex) {
        g.add_vertex(vertex);
    }

    // Notify the user that bus routes (edges) are being defined with distances (weights).
    cout << "\nDefining bus routes as edges with distances." << endl;
    // Define the bus routes as a vector of tuples, each representing an edge with a start vertex, an end vertex, and a weight.
    vector<tuple<char, char, int>> routes = {
        {'A', 'B', 50}, {'A', 'D', 150}, {'A', 'E', 200}, {'B', 'C', 40},
        {'B', 'E', 60}, {'C', 'G', 120}, {'D', 'E', 90}, {'E', 'F', 80},
        {'F', 'G', 70}, {'G', 'H', 100}, {'G', 'I', 60}, {'H', 'I', 80},
        {'H', 'J', 140}, {'I', 'J', 120}
    };
    // Add each edge defined in the routes vector to the graph.
    for (const auto& [start, end, distance] : routes) {
        g.add_edge(start, end, distance);
    }

    // Notify the user that the MST is being constructed and tested.
    cout << "\nConstructing and testing the Minimum Spanning Tree (MST):" << endl;
    // Construct the MST using the graph's min_span_tree method.
    auto mst = g.min_span_tree();

    // Calculate and display the total weight of the MST.
    int totalWeight = 0;
    for (const auto& [from, edges] : mst) {
        for (const auto& [to, weight] : edges) {
            totalWeight += weight; // Add the weight of each edge in the MST to the total weight.
            cout << from << " - " << to << " : " << weight << endl; // Display each edge and its weight.
        }
    }
    cout << "Total weight (miles) of the MST: " << totalWeight << endl; // Display the total weight of the MST.

    return 0; // End of the program.
}
