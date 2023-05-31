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




    void readEdges(const std::string &filename);
    void readTourism();
    void readRealNodes(const std::string &filename);
};


#endif //DA_SHIPPING_PROJECT_FILE_READER_H