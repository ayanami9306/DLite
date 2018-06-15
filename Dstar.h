#ifndef DSTAR_H
#define DSTAR_H

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>

#define MAP_SIZE 10

using namespace std;

class state {
public:
    int x;
    int y;
    pair<double,double> k;
    
    bool operator == (const state &s2) const {
        return ((x == s2.x) && (y == s2.y));
    }
    
    bool operator != (const state &s2) const {
        return ((x != s2.x) || (y != s2.y));
    }
    
    bool operator > (const state &s2) const {
        if (k.first-0.00001 > s2.k.first) return true;
        else if (k.first < s2.k.first-0.00001) return false;
        return k.second > s2.k.second;
    }
    
    bool operator <= (const state &s2) const {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + 0.00001;
    }
    
    
    bool operator < (const state &s2) const {
        if (k.first + 0.000001 < s2.k.first) return true;
        else if (k.first - 0.000001 > s2.k.first) return false;
        return k.second < s2.k.second;
    }
    
};

struct ipoint2 {
    int x,y;
};

struct cellInfo {
    
    double g;
    double rhs;
    
};

struct costInfo {
    double cost_up, cost_down, cost_left, cost_right;
};

class Dijkstra_Node
{
public:
    Dijkstra_Node(int lx, int ly, int ldist)
    {
        x = lx;
        y = ly;
        dist = ldist;
    }
    
    //grater operator for min heap
    bool operator > (const Dijkstra_Node& Node) const
    {
        return dist > Node.dist;
    }
    
    int x, y, dist;
};

class state_hash {
public:
    size_t operator()(const state &s) const {
        return s.x + 34245*s.y;
    }
};


typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
typedef unordered_map<state, cellInfo, state_hash, equal_to<state> > ds_ch;
typedef unordered_map<state, costInfo, state_hash, equal_to<state> > ds_co;
typedef unordered_map<state, float, state_hash, equal_to<state> > ds_oh;


class Dstar {
    
public:
    
    Dstar();
    void   init(int sX, int sY, int gX, int gY);
    void   updateCell(int x, int y, double * val);
    void   updateStart(int x, int y);
    void   updateGoal(int x, int y);
    void   updateWall(int x1, int y1, int x2, int y2);
    void   Dijkstra();
    bool   replan();
    
    list<state> getDijkstraPath();
    int getDijkstraCost();
    list<state> getPath();
    int getCost();
    
private:
    
    list<state> path;
    list<state> Dijkstra_Path;
    double DLite_Cost;
    double Dijkstra_Cost;
    
    double C1;
    double k_m;
    state s_start, s_goal, s_last;
    int maxSteps;
    
    ds_pq openList;
    ds_ch cellHash;
    ds_co costHash;
    ds_oh openHash;
    
    void   updateCell_with_Goal(int x, int y);
    bool   close(double x, double y);
    void   makeNewCell(state u);
    void   makeNewCell_with_Goal(state u);
    double getG(state u);
    double getRHS(state u);
    void   setG(state u, double g);
    void   setRHS(state u, double rhs);
    double fourCondist(state a, state b);
    int    computeShortestPath();
    void   updateVertex(state u);
    void   insert(state u);
    void   remove(state u);
    double trueDist(state a, state b);
    double heuristic(state a, state b);
    state  calculateKey(state u);
    void   getSucc(state u, list<state> &s);
    void   getPred(state u, list<state> &s);
    double cost(state a, state b);
    bool   isValid(state u);
    float  keyHashCode(state u);
    bool   Dijkstra_calc_distance(int (*dist)[MAP_SIZE], int (*prev)[MAP_SIZE][2], state v, Dijkstra_Node u, double cost);
    
};

#endif
