#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <algorithm>
#include <sstream>

class Graph {
public:
    using VertexType = std::string;
    using WeightType = double;

private:
    struct Edge {
        VertexType target;
        WeightType weight;
        Edge(VertexType t, WeightType w) : target(t), weight(w) {}
    };

    std::map<VertexType, std::vector<Edge>> adjList;

public:
    void addVertex(const VertexType& vertex) {
        if (adjList.find(vertex) == adjList.end()) {
            adjList[vertex] = std::vector<Edge>();
        }
    }

    void addEdge(const VertexType& source, const VertexType& target, WeightType weight) {
        addVertex(source);
        addVertex(target);
        adjList[source].emplace_back(target, weight);
        adjList[target].emplace_back(source, weight);
    }

    std::vector<VertexType> shortestPath(const VertexType& source, const VertexType& destination) {
        std::map<VertexType, WeightType> distances;
        std::map<VertexType, VertexType> predecessors;
        std::priority_queue<std::pair<WeightType, VertexType>, std::vector<std::pair<WeightType, VertexType>>, std::greater<>> queue;

        for (const auto& vertex : adjList) {
            distances[vertex.first] = std::numeric_limits<WeightType>::max();
        }
        distances[source] = 0;

        queue.push({0, source});

        while (!queue.empty()) {
            auto currentVertex = queue.top().second;
            queue.pop();

            if (currentVertex == destination) break;

            for (const auto& edge : adjList[currentVertex]) {
                WeightType distance = distances[currentVertex] + edge.weight;
                if (distance < distances[edge.target]) {
                    distances[edge.target] = distance;
                    predecessors[edge.target] = currentVertex;
                    queue.push({distance, edge.target});
                }
            }
        }

        std::vector<VertexType> path;
        for (VertexType at = destination; at != source; at = predecessors[at]) {
            if (predecessors.find(at) == predecessors.end()) {
                std::cout << "No path exists from " << source << " to " << destination << ".\n";
                return {};
            }
            path.push_back(at);
        }
        path.push_back(source);
        std::reverse(path.begin(), path.end());
        return path;
    }

    WeightType minSpanningTree(const VertexType& startVertex) {
        std::set<VertexType> inTree;
        std::priority_queue<std::pair<WeightType, VertexType>, std::vector<std::pair<WeightType, VertexType>>, std::greater<>> edges;
        WeightType totalWeight = 0;

        edges.push({0, startVertex});

        while (!edges.empty()) {
            auto [weight, vertex] = edges.top();
            edges.pop();

            if (inTree.find(vertex) != inTree.end()) continue;
            inTree.insert(vertex);
            totalWeight += weight;

            for (const auto& edge : adjList[vertex]) {
                if (inTree.find(edge.target) == inTree.end()) {
                    edges.push({edge.weight, edge.target});
                }
            }
        }

        return totalWeight;
    }
};

int main() {
    Graph g;

    // Populate the graph with vertices and edges
    g.addVertex("A");
    g.addVertex("B");
    g.addVertex("C");
    g.addVertex("D");
    g.addVertex("E");

    g.addEdge("A", "B", 4);
    g.addEdge("A", "C", 1);
    g.addEdge("C", "B", 2);
    g.addEdge("B", "E", 4);
    g.addEdge("C", "D", 4);
    g.addEdge("D", "E", 1);

    // Shortest path demonstration
    auto path = g.shortestPath("A", "E");
    std::cout << "Shortest path from A to E: ";
    if (!path.empty()) {
        for (const auto& vertex : path) {
            std::cout << vertex << " ";
        }
        std::cout << std::endl;
    }

    // Minimum spanning tree demonstration
    auto mstWeight = g.minSpanningTree("A");
    std::cout << "Total weight of the minimum spanning tree starting from A: " << mstWeight << std::endl;

    return 0;
}