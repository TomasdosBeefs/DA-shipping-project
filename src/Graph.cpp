// By: Gonçalo Leão

#include "Graph.h"
#include "VertexEdge.h"
#include "MutablePriorityQueue.h"

int Graph::getNumVertex() const {
    return vertexSet.size();
}

std::vector<Vertex *> Graph::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
Vertex * Graph::findVertex(const int &id) const {
    for (auto v : vertexSet)
        if (v->getId() == id)
            return v;
    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
int Graph::findVertexIdx(const int &id) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getId() == id)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
bool Graph::addVertex(const int &id) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
bool Graph::addEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}

bool Graph::addBidirectionalEdge(const int &sourc, const int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

double Graph::Dijkstra(Vertex *v1, Vertex *v2) {

    for (Vertex *v: this->vertexSet) {
        v->setVisited(false);
        v->setDist(INF);
    }

    std::priority_queue<Vertex *> pq;
    v1->setDist(0);
    pq.push(v1);

    while (!pq.empty()) {

        Vertex *v = pq.top();
        pq.pop();

        if(v == v2) return v2->getDist();

        for (Edge *e: v->getAdj()) {
            if (e->getOrig()->getDist() + e->getWeight() < e->getDest()->getDist()) {
                e->getDest()->setDist(e->getOrig()->getDist() + e->getWeight());
                pq.push(e->getDest());
            }
        }

    }
    return v2->getDist();

}

void Graph::createAdjMatrix(){
    adjMatrix.clear();
    adjMatrix = std::vector<std::vector<double>>(vertexSet.size(),std::vector<double>(vertexSet.size(),0));
    for(unsigned long i = 0; i < adjMatrix.size(); i++){
        for(unsigned long j = i+1; j < adjMatrix.size(); j++){
            double dist = Dijkstra(vertexSet[i],vertexSet[j]);
            adjMatrix[i][j] = dist;
            adjMatrix[j][i] = dist;
        }
    }
}

double Graph::exercise1(){
    createAdjMatrix();
    double best = INF;
    std::vector<int> vetor;

    for(int i=0; i <vertexSet.size(); i++){
        vetor.push_back(i);
    }

    do{
        double cur = adjMatrix[vetor.front()][vetor.back()];
        for(unsigned long i = 1; i<vetor.size(); i++){
            cur += adjMatrix[vetor[i-1]][vetor[i]];
            if(cur>best)
                i = vetor.size();
        }

        best = std::min(best, cur);

    } while (std::next_permutation(vetor.begin(), vetor.end()));

    return best;

}

double Graph::exercise2(){
    MutablePriorityQueue <Vertex> q;
    double out = 0;

    for (auto &i : vertexSet){
        i->setDist(INF);
        i->setVisited(false);
    }

    vertexSet[0]->setDist(0);
    q.insert(vertexSet[0]); //qual e que e aqui

    while(!q.empty()){
        auto cur = q.extractMin();
        out+= cur->getDist() *2;
        cur->setVisited(true);

        for(auto e:cur->getAdj()){
            if(!e->getDest()->isVisited()){
                if(e->getDest()->getDist()==INF){
                    e->getDest()->setDist(e->getWeight());
                    q.insert(e->getDest());
                }
                else if (e->getWeight() < e->getDest()->getDist())
                    e->getDest()->setDist(e->getWeight());
                    q.decreaseKey(e->getDest());
            }
        }
    }


    return out;

}

Graph::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}