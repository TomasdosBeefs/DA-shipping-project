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
    double Dijkstra(Vertex *v1, Vertex *v2);
    void createAdjMatrix();

    double haversine(double lat1, double lon1, double lat2, double lon2);

    //Exercises

    double exercise1();
    double exercise2();
    double exercise3();

protected:
    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    int findVertexIdx(const int &id) const;

    void DFS(int id, std::vector<Vertex *> &path, std::vector<std::vector<Edge *>> &mst, std::vector<bool>& visited);

    void Prims(std::vector<std::vector<Edge *>> &mst);

    double distance_calc(std::vector<Vertex *> &path);

    void complete_matrix();

    int Nearest_unvisited_vertex(std::vector<bool>& visited,int cur);

    std::vector<double> nearest_neighbor_tour(std::vector<bool>& visited);

    double get_distance(Vertex* v1, Vertex* v2);


    void nearest_neighbor_tour(std::vector<bool> &visited, std::vector<Vertex *> path);
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif //DA_SHIPPING_PROJECT_GRAPH_H