//
// Created by ritac on 24/05/2023.
//

#ifndef DA_SHIPPING_PROJECT_GRAPH_H
#define DA_SHIPPING_PROJECT_GRAPH_H
#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include "VertexEdge.h"

class Graph {
public:
    ~Graph();

    Vertex *findVertex(const int &id) const;

    bool addVertex(const int &id);

    bool addEdge(const int &sourc, const int &dest, double w);
    bool addBidirectionalEdge(const int &sourc, const int &dest, double w);

    int getNumVertex() const;
    std::vector<Vertex *> getVertexSet() const;

    std::vector<Vertex *> vertexSet;

    std::vector<std::vector <double>> adjMatrix;
    std::unordered_map<Vertex*, Vertex::vertexCoord> vertexCoordMap;


    //Auxiliary functions
    /**
 * Calculates the shortest distance between two vertices using Dijkstra's algorithm.
 *
 * @param v1 Pointer to the source vertex.
 * @param v2 Pointer to the destination vertex.
 * @return The shortest distance between v1 and v2.
 *
 * @complexity The time complexity of this function is O((V + E) * log(V)),
 *             where V is the number of vertices and E is the number of edges in the graph.
 */
    double Dijkstra(Vertex *v1, Vertex *v2);


/**
 * Creates the adjacency matrix representation of the graph.
 *
 * @complexity O(V^2)
 */
    void createAdjMatrix();


/**
 * Calculates the Haversine distance between two sets of latitude and longitude coordinates.
 *
 * @param lat1 Latitude of the first point.
 * @param lon1 Longitude of the first point.
 * @param lat2 Latitude of the second point.
 * @param lon2 Longitude of the second point.
 * @return The Haversine distance between the two points.
 *
 * @complexity O(1)
 */
    double haversine(double lat1, double lon1, double lat2, double lon2);


    //Exercises

    /**
 * Finds the optimal solution for the TSP using a backtracking algorithm.
 *
 * @return The length of the shortest path in the graph.
 *
 * @complexity The time complexity is O((V-1)!) and the space complexity is O(N)
 */
    double exercise1();


/**
 * Generates an approximation of the optimal solution for the TSP using the Triangular Approximation algorithm.
 *
 * @return The length of the generated tour.
 *
 * @complexity The time complexity is O(V^2 * log V) and the space complexity is O(N)
 */
    double exercise2();


/**
 * Generates an approximation of the optimal solution for the TSP using the Nearest Neighbor algorithm,
 * followed by a 2-opt local search optimization.
 *
 * @return The length of the generated tour after the 2-opt optimization.
 *
 * @complexity The time complexity is O(V^4) and the space complexity is O(V^2 * V)
 */
    double exercise3();


/**
 * Generates an approximation of the optimal solution for the TSP using a bitmask dynamic programming approach.
 *
 * @return The length of the generated tour.
 *
 * @complexity The time complexity is O((V-1) * 2^(V-1)) and the space complexity is O(N * 2^N)
 *
 */
    double exercise3_2();


protected:
    // vertex set

    std::vector<std::vector<double>>  distMatrix;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    int findVertexIdx(const int &id) const;

    /**
 * Performs a Depth-First Search (DFS) traversal starting from the given vertex ID.
 *
 * @param id The ID of the starting vertex for the DFS traversal.
 * @param path A vector to store the vertices visited during the traversal.
 * @param mst The Minimum Spanning Tree (MST) represented as a vector of vectors of edges.
 * @param visited A vector indicating whether each vertex has been visited.
 *
 * @complexity O(V + E)
 */
    void DFS(int id, std::vector<Vertex*>& path, std::vector<std::vector<Edge*>>& mst, std::vector<bool>& visited);


/**
 * Constructs a Minimum Spanning Tree (MST) using Prim's algorithm.
 *
 * @param mst A vector of vectors to store the MST as adjacency lists.
 *
 * @complexity O((V^2) * log(V)),
 */
    void Prims(std::vector<std::vector<Edge*>>& mst);


/**
 * Calculates the total distance of the given path.
 *
 * @param path A vector of vertices representing the path.
 *
 * @return The total distance of the path.
 *
 * @complexity O(V)
 */
    double distance_calc(std::vector<Vertex*>& path);


/**
 * Completes the distance matrix by calculating the distances between all pairs of vertices.
 *
 * @complexity O(V^2)
 */
    void complete_matrix();


/**
 * Finds the nearest unvisited vertex to the current vertex.
 *
 * @param visited A vector indicating whether each vertex has been visited.
 * @param cur The ID of the current vertex.
 *
 * @return The ID of the nearest unvisited vertex.
 *
 * @complexity O(V)
 */
    int Nearest_unvisited_vertex(std::vector<bool>& visited, int cur);


/**
 * Calculates the Haversine distance between two vertices using their latitude and longitude.
 *
 * @param v1 Pointer to the first vertex.
 * @param v2 Pointer to the second vertex.
 *
 * @return The Haversine distance between the two vertices.
 *
 * @complexity O(1).
 */
    double get_distance(Vertex* v1, Vertex* v2);


/**
 * Generates a tour using the Nearest Neighbor algorithm.
 *
 * @param visited A vector indicating whether each vertex has been visited.
 * @param path A vector to store the generated tour.
 *
 * @complexity O(V^2),
 */
    void nearest_neighbor_tour(std::vector<bool>& visited, std::vector<Vertex*>& path);


/**
 * Calculates the length of the tour.
 *
 * @param tour A vector of vertices representing the tour.
 *
 * @return The length of the tour.
 *
 * @complexity O(V)
 */
    double tour_length(std::vector<Vertex*>& tour);

    /**
 * Applies the 2-opt optimization to the given path.
 *
 * @param path A vector of vertices representing the path.
 *
 * @complexity O(V^2),
 *
 */
    void tsp_2opt(std::vector<Vertex*>& path);


/**
 * Recursive function to calculate the optimal solution for the Traveling Salesman Problem (TSP)
 * using dynamic programming and bitmasking.
 *
 * @param i The current vertex index.
 * @param mask The bitmask representing the set of visited vertices.
 *
 * @return The optimal cost of the TSP solution starting from vertex i and visiting all vertices in the mask.
 *
 * @complexity O(N^2 * 2^N)
 */
    double fun(int i, int mask);


    std::vector<std::vector<double>> memo;

    std::vector<std::vector<Vertex*>> paths;

/**
 * Recursive function to perform backtracking algorithm for the Traveling Salesman Problem (TSP).
 * It explores all possible paths starting from the current position and finds the optimal solution.
 *
 * @param currPos The current position (vertex) in the exploration.
 * @param count The count of visited vertices so far.
 * @param cost The total cost of the current path.
 * @param ans Reference to the variable storing the optimal cost found.
 * @param path Reference to the vector storing the optimal path found.
 *
 * @complexity O(N!)
 */
    void backtracking_algorithm_aux(int currPos, int count, double cost, double &ans, std::vector<Vertex*> &path);
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif //DA_SHIPPING_PROJECT_GRAPH_H