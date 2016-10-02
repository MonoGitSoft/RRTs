#include <iostream>
#include "node.h"
#include "rrts.h"
#include "vec2.h"
#include <fstream>


#define PI 3.141592

using namespace std;

vector< pair<double,double> >  MapGenerator(void) {
    pair<double,double> seed;
    vector< pair<double,double> > map;
    for(int i = 1; i < 80; i++) {
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
    /*seed.first = 80;
    seed.second = 80;
    map.push_back(seed);
    seed.first = -80;
    seed.second = -80;/
    map.push_back(seed);*/
    return map;
}

void test(void){
        vector< pair<double,double> > map;
        Node firstNode(0,0);
        Node J(10,10,&firstNode);
        Node B(-10,10,&firstNode);
        RRTs test(map,firstNode,100,100);
        test.graf.push_back(&J);
        test.graf.push_back(&B);
        Node rand(10,0);
        Node* closest;
        closest = test.ClosestNode(&rand);
        cout<<closest->x<<" "<<closest->y<<endl;
        cout<<closest->parent->x<<" "<<closest->parent->y<<endl;
        //cout<<closest->childern[0]->x<<" "<<closest->childern[0]->y<<endl;
        cout<<"asdasd"<<endl;
        if(!test.IsPartOfGraf(closest)) {
            test.InsertToGraf(closest);
        }
        rand.parent = closest;
        test.InsertToGraf(&rand);
        test.ExportGraf();

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

void CollisionTest(void) {
    ofstream myfile;
    ofstream mapfile;
    myfile.open("graf.txt");
    mapfile.open("map.txt");
    Node firstNode(0,0); //Node firstNode(12.82,-77.5);
    Node J(-10,180,&firstNode); //Node J(50,50,&firstNode);
    Node B(-200,200,&firstNode);//Node B(-187.83,-187.39,&firstNode);
    RRTs test(MapGenerator(),firstNode,500,500);
    //Obstacle obs(-80,-80);
    Obstacle obs(240,200);
    Obstacle obs_2(201.5,199.4);
    vector<Obstacle> os;
    os = test.obstacles;
   // os.push_back(obs_2);
    /*if(obs.IsCollision(firstNode,&J)) {
        cout<<"ütközés"<<endl;
        cout<<J.x<<" "<<J.y<<endl;
    }
    else {
        cout<<"nicsn asd"<<endl;
    }
    if(obs_2.IsCollision(firstNode,&J)) {
        cout<<"ütközés"<<endl;
        cout<<J.x<<" "<<J.y<<endl;
    }*/
    for(int i = 0; i < os.size(); i++) {
        cout<<"elötte: "<<J.x<<" "<<J.y<<endl;
        os[i].IsCollision(firstNode,&J);
        cout<<"utana: "<<J.x<<" "<<J.y<<endl;
    }
    myfile<<firstNode.x<<' '<<firstNode.y<<endl;
    myfile<<J.x<<' '<<J.y<<endl;
    myfile<<firstNode.x<<' '<<firstNode.y<<endl;
    myfile<<B.x<<' '<<B.y<<endl;
    for(int i = 0; i < os.size();i++) {
    mapfile<<os[i].x<<' '<<os[i].y<<endl;
    }
    // mapfile<<obs_2.x<<' '<<obs_2.y<<endl;
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
    vector< pair<double,double> > map;
    RRTs test(MapGenerator(),firstNode,500,500);
    test.PathPlaning(goal);
    test.ExportGraf();
    //CollisionTest();
    //VecTest();
    return 0;
}

