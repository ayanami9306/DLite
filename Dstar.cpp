#include "Dstar.h"

/* void Dstar::Dstar()
 * --------------------------
 * Constructor sets constants.
 */
Dstar::Dstar()
{
    
    maxSteps = 80000;  // node expansions before we give up
    C1       = 1;      // cost of an unseen cell
    
}

/* float Dstar::keyHashCode(state u)
 * --------------------------
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float Dstar::keyHashCode(state u)
{
    
    return (float)(u.k.first + 1193*u.k.second);
    
}

/* bool Dstar::isValid(state u)
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool Dstar::isValid(state u)
{
    
    ds_oh::iterator cur = openHash.find(u);
    if (cur == openHash.end()) return false;
    if (!close(keyHashCode(u), cur->second)) return false;
    return true;
    
}

/* void Dstar::getPath()
 * --------------------------
 * Returns the path created by replan()
 */
list<state> Dstar::getPath()
{
    
    return path;
    
}

list<state> Dstar::getDijkstraPath()
{
    
    return Dijkstra_Path;
    
}

int Dstar::getCost()
{
    
    return (int)DLite_Cost;
    
}

int Dstar::getDijkstraCost()
{
    
    return (int)Dijkstra_Cost;
    
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY)
{
    
    cellHash.clear();
    costHash.clear();
    path.clear();
    openHash.clear();
    while(!openList.empty()) openList.pop();
    
    k_m = 0;
    
    s_start.x = sX;
    s_start.y = sY;
    s_goal.x  = gX;
    s_goal.y  = gY;
    
    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    costInfo tmp_cost;
    tmp_cost.cost_up = C1;
    tmp_cost.cost_down = C1;
    tmp_cost.cost_left = C1;
    tmp_cost.cost_right = C1;
    
    cellHash[s_goal] = tmp;
    costHash[s_goal] = tmp_cost;
    
    tmp.g = tmp.rhs = heuristic(s_start,s_goal);
    cellHash[s_start] = tmp;
    costHash[s_start] = tmp_cost;
    s_start = calculateKey(s_start);
    
    s_last = s_start;
    
}

/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void Dstar::makeNewCell(state u)
{
    
    //is exist?
    if (cellHash.find(u) != cellHash.end()) return;
    
    cellInfo tmp;
    costInfo tmp_cost;
    tmp.g       = tmp.rhs = heuristic(u,s_goal);
    tmp_cost.cost_up    = C1;
    tmp_cost.cost_down  = C1;
    tmp_cost.cost_left  = C1;
    tmp_cost.cost_right = C1;
    cellHash[u] = tmp;
    costHash[u] = tmp_cost;
    
}

void Dstar::makeNewCell_with_Goal(state u)
{
    
    //is exist?
    if (cellHash.find(u) != cellHash.end()) return;
    
    cellInfo tmp;
    tmp.g       = tmp.rhs = heuristic(u,s_goal);
    cellHash[u] = tmp;
    
}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double Dstar::getG(state u)
{
    
    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal);
    return cellHash[u].g;
    
}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double Dstar::getRHS(state u)
{
    
    if (u == s_goal) return 0;
    
    if (cellHash.find(u) == cellHash.end())
        return heuristic(u,s_goal);
    return cellHash[u].rhs;
    
}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void Dstar::setG(state u, double g)
{
    
    makeNewCell(u);
    cellHash[u].g = g;
    
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
void Dstar::setRHS(state u, double rhs)
{
    
    makeNewCell(u);
    cellHash[u].rhs = rhs;
    
}

/* double Dstar::fourCondist(state a, state b)
 * --------------------------
 * Returns the 4-way distance between state a and state b.
 */
double Dstar::fourCondist(state a, state b)
{
    
    return fabs(a.x - b.x) + fabs(a.y - b.y);
    
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles.
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int Dstar::computeShortestPath() {
    
    list<state> s;
    list<state>::iterator i;
    
    if (openList.empty()) return 1;
    
    int k=0;
    while (
           !openList.empty()
           &&
           (( openList.top() < (s_start = calculateKey(s_start)) )
           ||
           ( getRHS(s_start) != getG(s_start) ))
           ) {
        
        if (k++ > maxSteps) {
            fprintf(stderr, "At maxsteps\n");
            return -1;
        }
        
        
        state u;
        
        bool test = (getRHS(s_start) != getG(s_start));
        
        // lazy remove
        while(1) {
            if (openList.empty()) return 1;
            u = openList.top();
            openList.pop();
            
            if (!isValid(u)) continue;
            if (!(u < s_start) && (!test)) return 2;
            break;
        }
        
        ds_oh::iterator cur = openHash.find(u);
        openHash.erase(cur);
        
        state k_old = u;
        
        if (k_old < calculateKey(u)) { // u is out of date
            insert(u);
        } else if (getG(   u) > getRHS(u)) { // needs update (got better)
            setG(u,getRHS(u));
            getPred(u,s);
            for (i=s.begin();i != s.end(); i++) {
                updateVertex(*i);
            }
        } else {   // g <= rhs, state has got worse
            setG(u,INFINITY);
            getPred(u,s);
            for (i=s.begin();i != s.end(); i++) {
                updateVertex(*i);
            }
            updateVertex(u);
        }
    }
    return 0;
}

/* bool Dstar::close(double x, double y)
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool Dstar::close(double x, double y)
{
    
    if (isinf(x) && isinf(y)) return true;
    return (fabs(x-y) < 0.00001);
    
}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateVertex(state u)
{
    
    list<state> s;
    list<state>::iterator i;
    
    if (u != s_goal)
    {
        getSucc(u,s);
        double tmp = INFINITY;
        double tmp2;
        
        for (i=s.begin();i != s.end(); i++) {
            tmp2 = getG(*i) + cost(u,*i);
            if (tmp2 < tmp) tmp = tmp2;
        }
        if (!close(getRHS(u),tmp)) setRHS(u,tmp);
    }
    
    if (!close(getG(u),getRHS(u))) insert(u);
    
}

/* void Dstar::insert(state u)
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void Dstar::insert(state u) {
    
    ds_oh::iterator cur;
    float csum;
    
    u    = calculateKey(u);
    cur  = openHash.find(u);
    csum = keyHashCode(u);
    // return if cell is already in list. TODO: this should be
    // uncommented except it introduces a bug, I suspect that there is a
    // bug somewhere else and having duplicates in the openList queue
    // hides the problem...
    //if ((cur != openHash.end()) && (close(csum,cur->second))) return;
    
    openHash[u] = csum;
    openList.push(u);
}

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void Dstar::remove(state u)
{
    
    ds_oh::iterator cur = openHash.find(u);
    if (cur == openHash.end()) return;
    openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b)
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double Dstar::trueDist(state a, state b)
{
    
    float x = (float)a.x - (float)b.x;
    float y = (float)a.y - (float)b.y;
    return sqrt(x * x + y * y);
    
}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 4-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double Dstar::heuristic(state a, state b)
{
    return fourCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state Dstar::calculateKey(state u)
{
    
    double val = fmin(getRHS(u),getG(u));
    
    u.k.first  = val + heuristic(u,s_start) + k_m;
    u.k.second = val;
    
    return u;
    
}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 4-way cost.
 */
double Dstar::cost(state a, state b)
{
    
    int xd = b.x - a.x;
    int yd = b.y - a.y;
    
    if (cellHash.count(a) == 0)
    {
        printf("ERROR!!!!!!!! Check your Code!!!!!!!!!!!!!");
        return C1;
    }
    
    if(xd == 1) return costHash[a].cost_right;
    else if(xd == -1) return costHash[a].cost_left;
    else if(yd == 1) return costHash[a].cost_down;
    else if(yd == -1) return costHash[a].cost_up;
    
    else
    {
        printf("ERROR!!!!! 8-way????");
        return 99999999;
    }
    
}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double * val)
{
    
    state u;
    
    u.x = x;
    u.y = y;
    
    makeNewCell(u);
    costHash[u].cost_up = val[0];
    costHash[u].cost_down = val[1];
    costHash[u].cost_left = val[2];
    costHash[u].cost_right = val[3];
    
    //if((u == s_start) || (u == s_goal)) return;
    
    updateVertex(u);
    
}

void Dstar::updateCell_with_Goal(int x, int y)
{
    
    state u;
    
    u.x = x;
    u.y = y;
    
    makeNewCell_with_Goal(u);
    
    //if((u == s_start) || (u == s_goal)) return;
    
    updateVertex(u);
    
}

void Dstar::updateWall(int x1, int y1, int x2, int y2)
{
    state u, v;
    u.x = x1;
    u.y = y1;
    v.x = x2;
    v.y = y2;
    int dx = x2 - x1, dy = y2 - y1;
    //right
    if(dx == 1)
    {
        costHash[u].cost_right = -1;
        costHash[v].cost_left = -1;
    }
    //left
    else if(dx == -1)
    {
        costHash[u].cost_left = -1;
        costHash[v].cost_right = -1;
    }
    else if(dy == 1)
    {
        costHash[u].cost_down = -1;
        costHash[v].cost_up = -1;
    }
    else if(dy == -1)
    {
        costHash[u].cost_up = -1;
        costHash[v].cost_down = -1;
    }
    
    //if((u == s_start) || (u == s_goal)) return;
    
    updateVertex(u);
    updateVertex(v);
}

/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 4-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors.
 */
void Dstar::getSucc(state u,list<state> &s) {
    
    s.clear();
    u.k.first  = -1;
    u.k.second = -1;
    
    //up
    if(costHash[u].cost_up != -1)
    {
        u.y--;
        s.push_front(u);
        u.y++;
    }
    //down
    if(costHash[u].cost_down != -1)
    {
        u.y++;
        s.push_front(u);
        u.y--;
    }
    //left
    if(costHash[u].cost_left != -1)
    {
        u.x--;
        s.push_front(u);
        u.x++;
    }
    //right
    if(costHash[u].cost_right != -1)
    {
        u.x++;
        s.push_front(u);
        u.x--;
    }
}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 4-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void Dstar::getPred(state u,list<state> &s) {
    
    s.clear();
    u.k.first  = -1;
    u.k.second = -1;
    
    //up
    if(costHash[u].cost_up != -1)
    {
        u.y--;
        s.push_front(u);
        u.y++;
    }
    //down
    if(costHash[u].cost_down != -1)
    {
        u.y++;
        s.push_front(u);
        u.y--;
    }
    //left
    if(costHash[u].cost_left != -1)
    {
        u.x--;
        s.push_front(u);
        u.x++;
    }
    //right
    if(costHash[u].cost_right != -1)
    {
        u.x++;
        s.push_front(u);
        u.x--;
    }
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void Dstar::updateStart(int x, int y) {
    
    s_start.x = x;
    s_start.y = y;
    
    k_m += heuristic(s_last,s_start);
    
    s_start = calculateKey(s_start);
    s_last  = s_start;
    
}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void Dstar::updateGoal(int x, int y) {
    
    //0:up, 1:down, 2:left, 3:right
    list<ipoint2> toAdd;
    ipoint2 tp;
    
    ds_ch::iterator i;
    list<ipoint2>::iterator kk;
    
    for(i=cellHash.begin(); i!=cellHash.end(); i++)
    {
        tp.x = i->first.x;
        tp.y = i->first.y;
        toAdd.push_back(tp);
    }
    
    cellHash.clear();
    openHash.clear();
    
    while(!openList.empty())
        openList.pop();
    
    k_m = 0;
    
    s_goal.x  = x;
    s_goal.y  = y;
    
    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    
    cellHash[s_goal] = tmp;
    
    tmp.g = tmp.rhs = heuristic(s_start,s_goal);
    
    cellHash[s_start] = tmp;
    s_start = calculateKey(s_start);
    
    s_last = s_start;
    
    for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
        updateCell_with_Goal(kk->x, kk->y);
    }
    
    
}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start).
 */
bool Dstar::replan(){
    
    path.clear();
    /*for(int i=0; i<10; i++)
     for(int j=0; j<10; j++)
     {
     state u;
     u.x = j;
     u.y = i;
     printf("%f %f %f %f\n",costHash[u].cost_up, costHash[u].cost_down, costHash[u].cost_left, costHash[u].cost_right);
     }*/
    
    
    int res = computeShortestPath();
    //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
    if (res < 0) {
        fprintf(stderr, "NO PATH TO GOAL\n");
        return false;
    }
    list<state> n;
    list<state>::iterator i;
    
    state cur = s_start;
    
    if (isinf(getG(s_start))) {
        fprintf(stderr, "NO PATH TO GOAL\n");
        return false;
    }
    
    while(cur != s_goal) {
        
        path.push_back(cur);
        getSucc(cur, n);
        
        if (n.empty()) {
            fprintf(stderr, "NO PATH TO GOAL\n");
            return false;
        }
        
        double cmin = INFINITY;
        double tmin = INFINITY;
        state smin;
        
        for (i=n.begin(); i!=n.end(); i++) {
            
            //if (occupied(*i)) continue;
            double val  = cost(cur,*i);
            double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
            val += getG(*i);
            
            if (close(val,cmin)) {
                if (tmin > val2) {
                    tmin = val2;
                    cmin = val;
                    smin = *i;
                }
            } else if (val < cmin) {
                tmin = val2;
                cmin = val;
                smin = *i;
            }
        }
        n.clear();
        cur = smin;
    }
    path.push_back(s_goal);
    
    list<state>::iterator iter = path.begin();
    state iter_cur;
    state iter_next;
    iter_cur.x = iter->x;
    iter_cur.y = iter->y;
    iter++;
    DLite_Cost = 0;
    for(; iter!=path.end(); iter++)
    {
        iter_next.x = iter->x;
        iter_next.y = iter->y;
        int dx = iter_next.x - iter_cur.x, dy = iter_next.y - iter_cur.y;
        if(dx == 1) DLite_Cost += costHash[iter_cur].cost_right;
            else if(dx == -1) DLite_Cost += costHash[iter_cur].cost_left;
                else if(dy == 1) DLite_Cost += costHash[iter_cur].cost_down;
                    else if(dy == -1) DLite_Cost += costHash[iter_cur].cost_up;
                        iter_cur = iter_next;
                        }
        return true;

}

bool Dstar::Dijkstra_calc_distance(int (*dist)[MAP_SIZE], int (*prev)[MAP_SIZE][2], state v, Dijkstra_Node u, double cost)
{
    int alt_distance = 0;
    if(cost != -1)
    {
        alt_distance = u.dist + cost;
        if(alt_distance < dist[v.x][v.y])
        {
            dist[v.x][v.y] = alt_distance;
            prev[v.x][v.y][0] = u.x;
            prev[v.x][v.y][1] = u.y;
        }
        return true;
    }
    return false;
}

void Dstar::Dijkstra()
{
    int dist[MAP_SIZE][MAP_SIZE] = {0, }, prev[MAP_SIZE][MAP_SIZE][2] = {-1, };
    bool is_visited[MAP_SIZE][MAP_SIZE] = {false, };
    priority_queue<Dijkstra_Node, vector<Dijkstra_Node>, greater<Dijkstra_Node> > Q;
    
    //set of unvisited node Q initialization
    for(int x=0; x<MAP_SIZE; x++)
        for(int y=0; y<MAP_SIZE; y++)
        {
            
            //if state is not start
            if((x != s_start.x) || (y != s_start.y))
            {
                //distance initialization, prev node initialization
                dist[x][y] = numeric_limits<int>::max();
                prev[x][y][0] = -1;
                prev[x][y][1] = -1;
            }
            
        }
    Q.push(Dijkstra_Node(s_start.x, s_start.y, 0));
    
    //loop until Q is empty
    Dijkstra_Node u(0, 0, 0);
    state state_u, v;
    while (!Q.empty())
    {
        u = Q.top();
        is_visited[u.x][u.y] = true;
        state_u.x = u.x;
        state_u.y = u.y;
        Q.pop();
        
        //up
        v.x = u.x;
        v.y = u.y - 1;
        if(Dijkstra_calc_distance(dist, prev, v, u, costHash[state_u].cost_up) && is_visited[v.x][v.y] == false)
        {
            Q.push(Dijkstra_Node(v.x, v.y, dist[v.x][v.y]));
        }
        
        
        //down
        v.x = u.x;
        v.y = u.y + 1;
        if(Dijkstra_calc_distance(dist, prev, v, u, costHash[state_u].cost_down) && is_visited[v.x][v.y] == false)
        {
            Q.push(Dijkstra_Node(v.x, v.y, dist[v.x][v.y]));
        }
        
        //left
        v.x = u.x - 1;
        v.y = u.y;
        if(Dijkstra_calc_distance(dist, prev, v, u, costHash[state_u].cost_left) && is_visited[v.x][v.y] == false)
        {
            Q.push(Dijkstra_Node(v.x, v.y, dist[v.x][v.y]));
        }
        
        //right
        v.x = u.x + 1;
        v.y = u.y;
        if(Dijkstra_calc_distance(dist, prev, v, u, costHash[state_u].cost_right) && is_visited[v.x][v.y] == false)
        {
            Q.push(Dijkstra_Node(v.x, v.y, dist[v.x][v.y]));
        }
    }
    
    //path calc
    state path_traveler;
    state prev_path;
    path_traveler.x = s_goal.x;
    path_traveler.y = s_goal.y;
    Dijkstra_Cost = 0;
    Dijkstra_Path.clear();
    while(1)
    {
        Dijkstra_Path.push_front(path_traveler);
        if(path_traveler == s_start) break;
        prev_path.x = prev[path_traveler.x][path_traveler.y][0];
        prev_path.y = prev[path_traveler.x][path_traveler.y][1];
        int dx = path_traveler.x - prev_path.x, dy = path_traveler.y - prev_path.y;
        if(dx == 1) Dijkstra_Cost += costHash[prev_path].cost_right;
        else if(dx == -1) Dijkstra_Cost += costHash[prev_path].cost_left;
        else if(dy == 1) Dijkstra_Cost += costHash[prev_path].cost_down;
        else if(dy == -1) Dijkstra_Cost += costHash[prev_path].cost_up;
        path_traveler = prev_path;
    }
}
