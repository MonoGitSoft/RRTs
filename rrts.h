#ifndef RRTS_H
#define RRTS_H

#include "node.h"
#include "vec2.h"
#include <utility>

class Obstacle {
public:
    Obstacle();
    Obstacle(double x, double y);
    bool IsCollision(Node inGraf, Node* randNode);
    void CircVecDist(Node* random, Node graf);
    double x;
    double y;
    const static double r = 10;
};

class RRTs {
public:
    RRTs(std::vector< std::pair<double,double> > map,Node firstNode,double maxX, double maxY);
    Node* ClosestNode(Node* randNode);
    std::vector< std::pair<double,double> > map;
    std::vector<Obstacle> obstacles;
    std::vector<Node*> graf;
    Node* RandNode(void);
    bool IsPartOfGraf(Node* newNode);
    void InsertToGraf(Node * newNode);
    void PathPlaning(Node goal);
    void ExportGraf();
    double maxMapSizeX;
    double maxMapSizeY;
    Node goal;
    std::vector<Node*> path;
};

#endif // RRTS_H
