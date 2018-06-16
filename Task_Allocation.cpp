#include "Task_Allocation.h"
#include "User_Defined_Function.h"
#include "MST.h"

extern bool Flag_Item_Activate[NUM_TASK];
extern Dstar * Task_to_Task[2];
extern Robot robotList[NUM_ROBOT];
extern vector<edge> MSTree[2];
extern coordinate itemCoord[NUM_TASK];

bool check_reservation(int robotID, int taskID)
{
    for (int i = 0; i < NUM_ROBOT; i++)
    {
        if (i != robotID)
        {
            if (robotList[i].AllocTask.taskId == taskID)
                return true;
        }
    }
    return false;
}
void alloc_new_task(int index)
{
    int current_bot = index;
    int to_task = robotList[current_bot].AllocTask.taskId;
    bool G1_List[2][NUM_TASK] = {false, };
    
    robotList[index].AllocTask.taskId = -1;
    
    //search tasks within 1G
    for(int tree_index = 0; tree_index < 2; tree_index++)
    {
        for(int element_i = 0; element_i < MSTree[tree_index].size(); element_i++)
        {
            if(MSTree[tree_index][element_i].index[0] == to_task)
            {
                int G1_Task = MSTree[tree_index][element_i].index[1];
                if(!check_reservation(current_bot, G1_Task))
                {
                    bool chk_G1_G1 = true;
                    for(int element_j = 0; element_j < MSTree[tree_index].size(); element_j++)
                    {
                        if(MSTree[tree_index][element_j].index[0] == G1_Task && MSTree[tree_index][element_j].index[1] != to_task)
                        {
                            int G1_G1_Task = MSTree[tree_index][element_j].index[1];
                            if(check_reservation(current_bot, G1_G1_Task)) chk_G1_G1 = false;
                        }
                        else if(MSTree[tree_index][element_j].index[1] == G1_Task && MSTree[tree_index][element_j].index[0] != to_task)
                        {
                            int G1_G1_Task = MSTree[tree_index][element_j].index[0];
                            if(check_reservation(current_bot, G1_G1_Task)) chk_G1_G1 = false;
                        }
                    }
                    if(chk_G1_G1)
                        G1_List[tree_index][G1_Task] = true;
                }
            }
            else if(MSTree[tree_index][element_i].index[1] == to_task)
            {
                int G1_Task = MSTree[tree_index][element_i].index[0];
                if(!check_reservation(current_bot, G1_Task))
                {
                    bool chk_G1_G1 = true;
                    for(int element_j = 0; element_j < MSTree[tree_index].size(); element_j++)
                    {
                        if(MSTree[tree_index][element_j].index[0] == G1_Task && MSTree[tree_index][element_j].index[1] != to_task)
                        {
                            int G1_G1_Task = MSTree[tree_index][element_j].index[1];
                            if(check_reservation(current_bot, G1_G1_Task)) chk_G1_G1 = false;
                        }
                        else if(MSTree[tree_index][element_j].index[1] == G1_Task && MSTree[tree_index][element_j].index[0] != to_task)
                        {
                            int G1_G1_Task = MSTree[tree_index][element_j].index[0];
                            if(check_reservation(current_bot, G1_G1_Task)) chk_G1_G1 = false;
                        }
                    }
                    if(chk_G1_G1)
                        G1_List[tree_index][G1_Task] = true;
                }
            }
        }
    }
    G1_List[0][to_task] = true;
    G1_List[1][to_task] = true;
    
    int min_cost = INF;
    int min_taskID = -1;
    for(int item_index=0; item_index<NUM_TASK; item_index++)
    {
        Change_Start(current_bot, item_index, robotList[current_bot].robotcoord.x, robotList[current_bot].robotcoord.y, 0);
        if(G1_List[0][item_index] & G1_List[1][item_index])
        {
            if(min_cost > getCost(current_bot, item_index, 0) + robotList[current_bot].taskCost[item_index])
            {
                min_taskID = item_index;
                min_cost = getCost(current_bot, item_index, 0) + robotList[current_bot].taskCost[item_index];
            }
        }
    }
    //greedy 할 필요없음. 최소한 자신의 job으로 할당됨
    
    vector<coordinate> tmp = getPath(index, min_taskID, 0);
    robotList[index].assignTask(min_taskID, tmp, itemCoord);
    robotList[index].pathIndex = 0;
    robotList[index].AllocTask.taskId = min_taskID;
    robotList[index].AllocTask.taskcoord = itemCoord[min_taskID];
    
    if(to_task != min_taskID)
    {
        printf("task re-Allocation\n");
        printf("robot %d => task %d (%d,%d)\n", index, min_taskID, itemCoord[min_taskID].x, itemCoord[min_taskID].y);
    }
    robotList[index].pre_path = getPath(index, robotList[index].AllocTask.taskId, 0);
}

void task_alloc()
{
    alloc alloc_robot[NUM_ROBOT];
    vector<task_to_task_cost> task_dis;
    task_to_task_cost task_dis_element;
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        for (int jj = 0; jj < NUM_TASK; jj++)
        {
            if (Flag_Item_Activate[jj] == true)
            {
                Change_Start(ii, jj, robotList[ii].robotcoord.x, robotList[ii].robotcoord.y, 0);
                task_dis_element.from = ii;
                task_dis_element.to = jj;
                task_dis_element.distance = getCost(ii, jj, 0) + robotList[ii].taskCost[jj];
                task_dis.push_back(task_dis_element);
            }
        }
    }
    
    sort(task_dis.begin(), task_dis.end(), cmp_task_task);
    
    for (int i = 0; i < task_dis.size(); i++)
    {
        bool chk = true;
        int current_bot = task_dis[i].from;
        if(robotList[current_bot].status != WORKING)
        {
            int to_task = task_dis[i].to;
            if(alloc_robot[current_bot].dis == INF)
            {
                //search tasks within 1G
                for(int tree_index = 0; tree_index < 2; tree_index++)
                {
                    for(int element_i = 0; element_i < MSTree[tree_index].size(); element_i++)
                    {
                        if(MSTree[tree_index][element_i].index[0] == to_task)
                        {
                            int G1_Task = MSTree[tree_index][element_i].index[1];
                            for(int robo_index = 0; robo_index < NUM_ROBOT; robo_index++)
                                if(robo_index != current_bot)
                                    if((alloc_robot[robo_index].taskID == G1_Task && alloc_robot[robo_index].dis != INF) || (alloc_robot[robo_index].taskID == to_task && alloc_robot[robo_index].dis != INF))
                                    {
                                        chk = false;
                                        break;
                                    }
                        }
                        else if(MSTree[tree_index][element_i].index[1] == to_task)
                        {
                            int G1_Task = MSTree[tree_index][element_i].index[0];
                            for(int robo_index = 0; robo_index < NUM_ROBOT; robo_index++)
                                if(robo_index != current_bot)
                                    if((alloc_robot[robo_index].taskID == G1_Task && alloc_robot[robo_index].dis != INF) || (alloc_robot[robo_index].taskID == to_task && alloc_robot[robo_index].dis != INF))
                                    {
                                        chk = false;
                                        break;
                                    }
                        }
                        if(!chk) break;
                    }
                    if(!chk) break;
                }
                
                if(chk)
                {
                    alloc_robot[current_bot].taskID = to_task;
                    alloc_robot[current_bot].dis = task_dis[i].distance;
                    task_dis[i].distance = INF;
                }
            }
        }
        else
        {
            alloc_robot[current_bot].taskID = robotList[current_bot].AllocTask.taskId;
            alloc_robot[current_bot].dis = robotList[current_bot].AllocTask.taskCost;
            task_dis[i].distance = INF;
        }
    }
    
    //not matched? greedy
    for(int i = 0; i < task_dis.size(); i++)
    {
        if(task_dis[i].distance != INF && alloc_robot[task_dis[i].from].dis == INF)
        {
            int current_bot = task_dis[i].from;
            int to_task = task_dis[i].to;
            bool chk = true;
            for(int robo_index = 0; robo_index < NUM_ROBOT; robo_index++)
                if(alloc_robot[robo_index].taskID == to_task)
                {
                    chk = false;
                    break;
                }
            if(chk)
            {
                alloc_robot[current_bot].taskID = to_task;
                alloc_robot[current_bot].dis = task_dis[i].distance;
            }
        }
    }
    
    printf("\n\ntask Allocation\n");
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        if(robotList[ii].status != WORKING)
        {
            robotList[ii].AllocTask.taskId = alloc_robot[ii].taskID;
            robotList[ii].AllocTask.taskcoord = itemCoord[alloc_robot[ii].taskID];
            robotList[ii].assignTask(alloc_robot[ii].taskID, getPath(ii, alloc_robot[ii].taskID, 0), itemCoord);
            printf("robot %d => task %d (%d,%d)\n", ii, alloc_robot[ii].taskID, itemCoord[alloc_robot[ii].taskID].x, itemCoord[alloc_robot[ii].taskID].y);
            robotList[ii].pre_path = getPath(ii, robotList[ii].AllocTask.taskId, 0);
        }
        else
        {
            printf("robot %d is working now\n", ii);
        }
    }
}
