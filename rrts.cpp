#include "rrts.h"
#include <cmath>
#include "vec2.h"
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

#define PI 3.141592

using namespace std;

Obstacle::Obstacle(double x, double y): x(x), y(y) {
}

Obstacle::Obstacle(): x(), y(){
}

void Obstacle::CircVecDist(Node* random, Node graf) {
    Vec2 newRand(*random,graf);
    cout<<"Circ"<<endl;
    newRand.Print();
    double R = r + 0.1;
    Vec2 PO(x,y);
    Vec2 PR(*random);
    Vec2 PG(graf);
    Vec2 GR = PR - PG;
    Vec2 RG = GR * (-1);
    Vec2 RO = PO - PR;
    Vec2 GRNorm = GR.Norm();
    newRand = newRand - (GRNorm * (R + 0.1));
    Vec2 GO = PO - PG;
    double dAlfa = SubtendedAngle(RG,RO,GO);
    if(dAlfa > (90 * PI/180)) {
        double dBeta = 180 * PI/180 - dAlfa;
        newRand = newRand + GRNorm*((R - sqrt(pow(R,2) - pow(sin(dBeta)*RO.Lenght(),2))) + RO.Lenght()*cos(dBeta));
        newRand = newRand + PG;
        random->x = newRand.x;
        random->y = newRand.y;
        return;
    }
    else {
        newRand = newRand + GRNorm*((r - sqrt(pow(r,2) - pow(sin(dAlfa)*RO.Lenght(),2))) - RO.Lenght()*cos(dAlfa));
        newRand = newRand + PG;
        random->x = newRand.x;
        random->y = newRand.y;
        return;
    }
}

double sgn(double sign) {
    if(sign < (90*PI/180)) {
        return 1;
    }
    else {
        return -1;
    }
}

bool Obstacle::IsCollision(Node inGraf, Node *randNode,bool edit) {
    Vec2 F(inGraf);
    Vec2 O(x,y);
    Vec2 R(*randNode);
    Vec2 RF = F - R;
    Vec2 RO = O - R;
    Vec2 FO = O - F;
    Vec2 newNode;
    double cosAlfa = SubtendedCos(FO,RF,RO);
    double cosBeta = SubtendedCos(RO,RF,FO);
    double sinAlfa = sqrt(1 - pow(cosAlfa,2));
    double sinBeta = sqrt(1 -pow(cosBeta,2));
    if(edit) {
        if((((RO.Lenght() <= r) && (cosBeta < 0) ) || (( sinAlfa*FO.Lenght() <= r) && (cosBeta > 0))) && (cosAlfa > 0)) {
            newNode = R + RF.Norm()*(r + RO.Lenght()*cosBeta - (r - sqrt(pow(r,2) - pow(sinBeta*RO.Lenght(),2))));
            randNode->x = newNode.x;
            randNode->y = newNode.y;
            return true;
        }
        return false;
    }
    else {
        if((((RO.Lenght() <= r*0.99) && (cosBeta < 0) ) || (( sinAlfa*FO.Lenght() <= r*0.99) && (cosBeta > 0))) && (cosAlfa > 0)) {
            return true;
        }
        return false;
    }
}


RRTs::RRTs(std::vector<std::pair<double,double> > map,Node firstNode ,double maxX, double maxY): maxMapSizeX(maxX),
    maxMapSizeY(maxY), graf(), path(), reducedPath() {
    this->map = map;
    Obstacle temp;
    Node* first = new Node[1];
    *first = firstNode;
    graf.push_back(first);
    for(int i = 0; i < map.size(); i++) {
        temp.x = map[i].first;
        temp.y = map[i].second;
        obstacles.push_back(temp);
    }
    srand (time(NULL));
}

bool RRTs::IsPartOfGraf(Node *newNode) {
   return newNode->partOf;
}

void RRTs::InsertToGraf(Node *newNode) {// elötte isPartOfGraf-fal ellenörnizni, hogy eleme-e
    graf.push_back(newNode);
    newNode->partOf = true;
    if(newNode->IsLeaf()) {
        newNode->parent->childern.push_back(newNode);
    }
    else {
        newNode->parent->childern.push_back(newNode);
        std::vector<Node*>::iterator it;
        it = find(newNode->parent->childern.begin(),newNode->parent->childern.end(),newNode->childern[0]);
        if(it == newNode->parent->childern.end()) {
            cout<<newNode->parent->childern.size()<<endl;
            cout<<newNode->childern.size()<<endl;
        }
        newNode->parent->childern.erase(it);
    }
}


Node* RRTs::ClosestNode(Node *randNode) {
    std::pair<Node*,double> newNodeCandidate;
    Node * minNode = graf[0];
    double minDist = sqrt(pow(maxMapSizeX,2) + pow(maxMapSizeY,2));
    for(int i = 0; i < graf.size(); i++) {
        for(int j = 0; j < graf[i]->childern.size(); j++) {
            newNodeCandidate = DistFromGraf(graf[i], graf[i]->childern[j],randNode);
            if(newNodeCandidate.second < minDist) {
                minDist = newNodeCandidate.second;
                minNode = newNodeCandidate.first;
            }
        }
    }
    return minNode;
}

Node* RRTs::RandNode() {
    Node* randNode;
    int choice = rand() % 100;
    if( choice < 90) {
        randNode = new Node;
        randNode->x = (double)((rand() % (int)(200*maxMapSizeX)) - (100*maxMapSizeX))/100;
        randNode->y = (double)((rand() % (int)(200*maxMapSizeY)) - (100*maxMapSizeY))/100;
    }
    else {
        randNode = new Node(goal.x,goal.y);
    }
}

void RRTs::PathPlaning(Node goal) {
    this->goal = goal;
    Node* randNode;
    Node* closestNode;
    do {
        randNode = RandNode();
        closestNode = ClosestNode(randNode);
        if(!IsPartOfGraf(closestNode)) {
            InsertToGraf(closestNode);
        }
        randNode->parent = closestNode;
        for(int i = 0; i < obstacles.size(); i++) {
            if(obstacles[i].IsCollision(*closestNode,randNode)) {
            }
        }
        InsertToGraf(randNode);
    }while( !((randNode->x == goal.x) && (randNode->y == goal.y)));

    Node* iter = randNode;

    while(iter != NULL){
        path.push_back(iter);
        iter = iter->parent;
    }
    iter = path[0];
    Node *tempbegin;
    Node *tempfront;
    Node *reduce;
    reducedPath.push_back(*path[0]);
    std::vector<Node*>::iterator begin = path.begin();
    std::vector<Node*>::iterator front = begin + 2;
    std::vector<Node*>::iterator good = begin + 1;
    bool crash = false;
    while(good != (path.end() - 1)) {
        for(begin; front != path.end(); front++) {
            tempbegin = *begin;
            for(int i = 0; i < obstacles.size(); i++) {
                if(obstacles[i].IsCollision(*tempbegin,*front,false)) {
                    crash = true;
                    break;
                }
            }
            if(!crash) {
                good = front;
            }
            crash = false;
        }
        begin = good;
        front = begin + 1;
        reducedPath.push_back(**good);
    }
}

void RRTs::ExportGraf() {
     ofstream myfile;
     ofstream mapfile;
     ofstream pathfile;
     ofstream reducedPathfile;
     myfile.open("graf.txt");
     mapfile.open("map.txt");
     pathfile.open("path.txt");
     reducedPathfile.open("reducedpath.txt");
     for(int i = 0; i < graf.size(); i++) {
         for(int j = 0; j < graf[i]->childern.size(); j++) {
                 myfile<<graf[i]->x<<" "<<graf[i]->y<<endl;
                 myfile<<graf[i]->childern[j]->x<<" "<<graf[i]->childern[j]->y<<endl;
             }
     }
     for(int i = 0; i < map.size(); i++) {
         mapfile<<map[i].first<<" "<<map[i].second<<endl;
     }
     for(int i = 0; i < path.size(); i++) {
         pathfile<<path[i]->x<<" "<<path[i]->y<<endl;
     }
     for(int i = 0; i < reducedPath.size(); i++) {
         reducedPathfile<<reducedPath[i].x<<" "<<reducedPath[i].y<<endl;
     }
}
