#include <iostream>
#include "node.h"
#include "rrts.h"
#include "vec2.h"
#include <fstream>


#define PI 3.141592

using namespace std;

vector< AncientObstacle* >  MapGenerator(void) {
    pair<double,double> seed;
    vector< pair<double,double> > map;
   /* for(int i = 1; i < 80; i++) {
        seed.second = i*10 - 300;
        seed.first = -200;
        map.push_back(seed);
    }
    for(int i = 1; i < 70; i++) {
        seed.second = i*10 - 500;
        seed.first = 0;
        map.push_back(seed);
    }
    for(int i = 1; i < 80; i++) {
        seed.second = i*10 - 300;
        seed.first = 200;
        map.push_back(seed);
    }
    for(int i = 1; i < 20; i++) {
        seed.second = 0;
        seed.first = 300 + i*10;
        map.push_back(seed);
    }
    for(int i = 1; i < 40; i++) {
        seed.first = 300;
        seed.second = i*10;
        map.push_back(seed);
    }
    for(int i = 1; i < 20; i++) {
        seed.first = -200 - i*10;
        seed.second = -300;
        map.push_back(seed);
    }*/
    Vec2 a(-200,-300);
    Vec2 b(-200,500);
    Vec2 c(0,-500);
    Vec2 d(0,200);
    Vec2 e(-100,0);
    Vec2 f(200,300);
    Vec2 g(200,250);
    Vec2 h(-50,300);
    Vec2 k(300,500);
    Vec2 l(300,-200);
    AncientObstacle* newobs;
    StraightObstacle* newtemp;
    vector<AncientObstacle*> obstacles;
    newobs = new StraightObstacle(a,b);
    obstacles.push_back(newobs);
    newobs = new StraightObstacle(c,d);
    obstacles.push_back(newobs);
    newobs = new StraightObstacle(e,f);
    obstacles.push_back(newobs);
    newobs = new StraightObstacle(g,h);
    obstacles.push_back(newobs);
    newobs = new StraightObstacle(k,l);
    obstacles.push_back(newobs);
    return obstacles;
}

void DistTest() {
    Node a(0,0);
    Node b(10,0,&a);
    Node rand(15,0);
    Node *result;
    std::pair<Node*,double> res;
    res = DistFromGraf(&a,&b,&rand);
    result = res.first;
    cout<<result->x<<" "<<result->y<<endl;
    cout<<result->parent->childern[0]->x<<" "<<result->parent->childern[0]->y<<endl;
   // cout<<result->childern[0]->x<<" "<<result->childern[0]->y<<endl;
}

void VecTest() {
    Vec2 a(0,0);
    Vec2 b(200,200);
    Vec2 c(250,250);
    Vec2 AC = c-a;
    Vec2 AB = b-a;
    Vec2 BC = c-b;
    AC.Print();
    AB.Print();
    BC.Print();
    double alfa = SubtendedAngle(AC,AB,BC);
    double beta = SubtendedAngle(AB,BC,AC);
    double gamma = SubtendedAngle(AC,BC,AB);
    cout<<alfa*(180/PI)<<" "<<beta*(180/PI)<<" "<<gamma*(180/PI)<<endl;
}

int main(int argc, char *argv[])
{
    Node firstNode(-400,200);
    Node goal(400,200);
    firstNode.partOf = true;
    vector< AncientObstacle* > temp;
    RRTs test(MapGenerator(),firstNode,500,500);
    test.PathPlaning(goal);
    test.ExportGraf();
    //CollisionTest();
    //VecTest();
   /* Vec2 f(-10,0);
    Vec2 e(10,0);
    Node a(2,-10);
    Node b(25,5);
    StraightObstacle o(f,e);
    o.IsCollision(a,&b);
    o.IsCollision(a,&b);
    cout<<b.x<<b.y<<endl;*/
    return 0;
}

