//
// Created by ritac on 24/05/2023.
//

#ifndef DA_SHIPPING_PROJECT_FILE_READER_H
#define DA_SHIPPING_PROJECT_FILE_READER_H

#include <unordered_set>
#include <unordered_map>
#include "Graph.h"
#include <chrono>


class File_Reader {

public:
    File_Reader();

    Graph graph;

    std::unordered_set<Vertex*> allVertexes;
    std::unordered_set<Edge*> allEdges;
    std::unordered_map<std::string, Vertex*> vertexNames;


    /**
 * @brief Reads the edges information from a file and populates the graph accordingly.
 *
 * This function reads the edges information from the specified file and creates the corresponding vertices and edges
 * in the graph. It expects the file to be in CSV format with each line representing an edge and containing the source
 * vertex ID, destination vertex ID, and edge weight. It handles the creation of new vertices and edges, and updates
 * the graph accordingly.
 *
 * @param filename The name of the file to read the edges information from.
 */
    void readEdges(const std::string& filename);

/**
 * @brief Reads the tourism information from a file and populates the graph accordingly.
 *
 * This function reads the tourism information from the specified file and creates the corresponding vertices and edges
 * in the graph. It expects the file to be in CSV format with each line representing an edge and containing the source
 * vertex ID, destination vertex ID, edge weight, label for the source vertex, and label for the destination vertex.
 * It handles the creation of new vertices and edges, and updates the graph accordingly.
 */
    void readTourism();

/**
 * @brief Reads the real-world nodes information from a file and updates the vertex coordinates in the graph.
 *
 * This function reads the real-world nodes information from the specified file and updates the vertex coordinates in
 * the graph. It expects the file to be in CSV format with each line representing a vertex and containing the vertex ID,
 * longitude, and latitude. It updates the corresponding vertex objects in the graph with the latitude and longitude
 * information.
 *
 * @param filename The name of the file to read the real-world nodes information from.
 */
    void readRealNodes(const std::string& filename);

};


#endif //DA_SHIPPING_PROJECT_FILE_READER_H