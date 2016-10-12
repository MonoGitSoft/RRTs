#ifndef RRTS_H
#define RRTS_H

#include "node.h"
#include "vec2.h"
#include <utility>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

enum obstacleType { pointObstacle, lineObstacle };

class AncientObstacle {
public:
    virtual bool IsCollision(Node inGraf, Node* randNode, bool edit = true) = 0;
    virtual void Draw(std::ofstream& out) = 0;
    int type;
};

class Obstacle: public AncientObstacle {
public:
    Obstacle();
    Obstacle(double x, double y);
    bool IsCollision(Node inGraf, Node* randNode, bool edit = true);
    void Draw(std::ofstream& out);
    void CircVecDist(Node* random, Node graf);
    double x;
    double y;
    const static double r = 10;
};

class StraightObstacle: public AncientObstacle {
public:
    StraightObstacle();
    StraightObstacle(Vec2 firstPoint, Vec2 endpoint);
    bool IsCollision(Node inGraf, Node* randNode, bool edit = true);
    void Draw(std::ofstream& out);
    bool Between(Vec2 m);
    Vec2 firstPoint;
    Vec2 endPoint;
    Vec2 firstUp;
    Vec2 firstDown;
    Vec2 endUp;
    Vec2 endDown;
    const static double r = 10;
};

class RRTs {
public:
    RRTs(std::vector<AncientObstacle*> map,Node firstNode,double maxX, double maxY);
    Node* ClosestNode(Node* randNode);
    std::vector<AncientObstacle*> obstacles;
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
    std::vector<Node> reducedPath;
    std::vector<Vec2> sendPath;
    std::vector<Node> dijkPath;
};

#endif // RRTS_H
