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


    //Auxiliary functions
    double Dijkstra(Vertex *v1, Vertex *v2);
    void createAdjMatrix();

    //Exercises

    double exercise1();

protected:
    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    int findVertexIdx(const int &id) const;

};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif //DA_SHIPPING_PROJECT_GRAPH_H