#include "MST.h"
#include "User_Defined_Function.h"
#include <algorithm>

extern vector<edge> MSTree[2];
extern bool Flag_Item_Activate[NUM_TASK];
extern coordinate itemCoord[NUM_TASK];

bool cmp_task_task(task_to_task_cost a, task_to_task_cost b)
{
    return a.distance < b.distance;
}

int find(int u, int *parent)
{
    if(u == parent[u])
        return u;
    return parent[u] = find(parent[u], parent);
}

bool merge(int u, int v, int *parent)
{
    u = find(u, parent);
    v = find(v, parent);
    
    if(u == v)
        return false;
    parent[u] = v;
    return true;
}

void draw_MSTree()
{
    vector<task_to_task_cost> task_graph;
    int parent[NUM_TASK];
    task_to_task_cost task_graph_element;
    for (int map_type = 0; map_type < 2; map_type++)
    {
        
        //loop about total item
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            //is item activated?
            if (Flag_Item_Activate[ii] == true)
            {
                Change_Start(map_type, 0, itemCoord[ii].x, itemCoord[ii].y, 1);
                for (int jj = ii + 1; jj < NUM_TASK; jj++)
                {
                    if(Flag_Item_Activate[jj] == true)
                    {
                        Change_Goal(map_type, 0, itemCoord[jj].x, itemCoord[jj].y, 1);
                        task_graph_element.from = ii;
                        task_graph_element.to = jj;
                        task_graph_element.distance = getCost(map_type, 0, 1);
                        task_graph.push_back(task_graph_element);
                    }
                }
            }
        }
        
        MSTree[map_type].clear();
        
        //sort task_graph order by ascending
        sort(task_graph.begin(), task_graph.end(), cmp_task_task);
        
        //parent initialization
        for(int i = 0; i < NUM_TASK; i++)
            parent[i] = i;
        
        //construct MST
        edge temp_element;
        for(int i = 0; i < task_graph.size(); i++)
        {
            if(merge(task_graph[i].from, task_graph[i].to, parent))
            {
                temp_element.dis = task_graph[i].distance;
                temp_element.index[0] = task_graph[i].from;
                temp_element.index[1] = task_graph[i].to;
                MSTree[map_type].push_back(temp_element);
            }
        }
    }
}
