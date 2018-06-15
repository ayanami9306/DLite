#define _CRT_SECURE_NO_WARNINGS    // 'fopen', 'fscanf' error handle

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "User_Defined_Function.h"

int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int Meet_hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int Meet_vWallMatrix[MAP_SIZE][MAP_SIZE - 1];

Robot robotList[NUM_ROBOT];
coordinate itemCoord[NUM_TASK];
bool Flag_Item_Activate[NUM_TASK] = { false, };
Dstar * Task_to_Task[2];
int task_produced = 0;

void print_result(Robot  * robotList, int mode)
{
    int totalConsumption = 0;
    
    printf("\n<results>\n");
    
    
    totalConsumption = 0;
    
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        printf("robot %d energy : %d\n", ii, robotList[ii].totalCost);
        
        totalConsumption += robotList[ii].totalCost;
    }
    printf("energy consumption total : %d\n", totalConsumption);
    
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        printf("robot %d path length : %d\n", ii, robotList[ii].totalBlocks);
        
    }
    printf("\n");
}

typedef struct edge
{
    coordinate* node[2];
    double dis;
}edge;
typedef struct alloc_task
{
    int taskID;
    double dis = INF;
}alloc;

bool tree[NUM_TASK][NUM_TASK] = { false, };            // task들의 연결을 확인
std::vector<edge> MSTree[2];                        // 각각 uniform 맵으로부터 만들어진 mst, random 맵으로부터 만들어진 mst
// MSTree[0] : Type1, MSTree[1] : Type2
// 0&2번 로봇이 random => 0번 맵, 1&3번 로봇이 uniform => 1번 맵

int check_tree(int start, int end, int pre = INF)
{
    //printf("check(%d,%d)\n", start, end);
    if (tree[start][end] == true)
        return 1;
    for (int i = 0; i < NUM_TASK; i++)
    {
        if (tree[start][i] == true && (i != pre))
        {
            if (check_tree(i, end, start)) return 1;
        }
    }
    return 0;
}

void clear_tree()
{
    for (int i = 0; i < NUM_TASK; i++)
        for (int j = 0; j < NUM_TASK; j++)
            tree[i][j] = false;
}

// 초기화에만 false 입력, 2개 맵으로 부터의 mst는 동시에 업뎃
void draw_MSTree()
{
    for (int map_type = 0; map_type < 2; map_type++)
    {
        double task_graph[NUM_TASK][NUM_TASK] = { INF, };    // 각 task들 사이의 cost 값
        
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
                        task_graph[ii][jj] = getCost(map_type, 0, 1);
                        task_graph[jj][ii] = getCost(map_type, 0, 1);
                    }
                    else
                    {
                        task_graph[ii][jj] = INF;
                        task_graph[jj][ii] = INF;
                    }
                }
            }
            else
            {
                for (int jj = 0; jj < NUM_TASK; jj++)
                {
                    task_graph[ii][jj] = INF;
                    task_graph[jj][ii] = INF;
                }
            }
        }
        
        int task_cnt = 0;
        for (int i = 0; i < task_produced; i++)
            if (Flag_Item_Activate[i] == true)
                task_cnt++;
        
        clear_tree();
        MSTree[map_type].clear();
        double mini = INF;
        //printf("\n<map type> %d\n", map_type);
        while (MSTree[map_type].size() < task_cnt - 1)
        {
            int tmp_ii, tmp_jj;
            for (int ii = 0; ii < NUM_TASK; ii++)
            {
                for (int jj = ii; jj < NUM_TASK; jj++)
                {
                    if (task_graph[ii][jj] < mini)
                    {
                        if (!check_tree(ii, jj))
                        {
                            mini = task_graph[ii][jj];
                            tmp_ii = ii;
                            tmp_jj = jj;
                        }
                    }
                }
            }
            edge minimum;
            minimum.node[0] = &itemCoord[tmp_ii];
            minimum.node[1] = &itemCoord[tmp_jj];
            minimum.dis = mini;
            MSTree[map_type].push_back(minimum);
            tree[tmp_ii][tmp_jj] = true;
            tree[tmp_jj][tmp_ii] = true;
            mini = INF;
            //printf("PUSH : %d(%d,%d) - %d(%d,%d)\n",tmp_ii, itemCoord[tmp_ii].x, itemCoord[tmp_ii].y, tmp_jj, itemCoord[tmp_jj].x, itemCoord[tmp_jj].y);
        }
    }
    printf("draw MSTree Fin.\n");
}

bool check_task(int robotID, int taskID, alloc* robo)
{
    if (robo[robotID].dis != INF)
        return false;
    for (int i = 0; i < NUM_ROBOT; i++)
    {
        if (robo[i].taskID == taskID)
            return false;
    }
    int linked_set = 0;
    for (int i = 0; i < NUM_TASK; i++)
    {
        if (tree[taskID][i] == true)
        {
            for (int j = 0; j < NUM_ROBOT; j++)
            {
                if (robo[j].taskID == i)
                    linked_set++;
                if (linked_set == 2)
                    return false;
            }
        }
    }
    return true;
}
// 초기화에만 false 입력
void task_alloc()
{
    alloc robot[NUM_ROBOT];
    double task_dis[NUM_ROBOT][NUM_TASK];
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        for (int jj = 0; jj < NUM_TASK; jj++)
        {
            if (Flag_Item_Activate[jj] == true)
            {
                Change_Start(ii % 2, jj, robotList[ii].robotcoord.x, robotList[ii].robotcoord.y, 0);
                Change_Goal(ii % 2, jj, itemCoord[jj].x, itemCoord[jj].y, 0);
                task_dis[ii][jj] = getCost(ii, jj, 0) + robotList[ii].taskCost[jj];
                
            }
            else
                task_dis[ii][jj] = INF;
        }
    }
    
    for (int robo = 0; robo < NUM_ROBOT; robo++)
    {
        int min = INF;
        int tmp_ii, tmp_jj;
        for (int ii = 0; ii < NUM_ROBOT; ii++)
        {
            for (int jj = 0; jj < NUM_TASK; jj++)
            {
                if (task_dis[ii][jj] < min && check_task(ii, jj, robot))
                {
                    min = task_dis[ii][jj];
                    tmp_ii = ii;
                    tmp_jj = jj;
                }
            }
        }
        robot[tmp_ii].taskID = tmp_jj;
        robot[tmp_ii].dis = min;
        min = INF;
    }
    
    printf("task Allocation\n");
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        robotList[ii].AllocTask.taskId = robot[ii].taskID;
        robotList[ii].AllocTask.taskcoord = itemCoord[robot[ii].taskID];
        robotList[ii].assignTask(robot[ii].taskID, getPath(ii, robot[ii].taskID, 0), itemCoord);
        printf("robot %d => task %d (%d,%d)\n", ii, robot[ii].taskID, itemCoord[robot[ii].taskID].x, itemCoord[robot[ii].taskID].y);
    }
}

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
    int min_taskID;
    double min_cost = INF;
    for (int ii = 0; ii < NUM_TASK; ii++)
    {
        if (tree[robotList[index].AllocTask.taskId][ii] && !check_reservation(index, ii))
        {
            if (min_cost > getCost(index, ii, 0))
            {
                min_taskID = ii;
                min_cost = getCost(index, ii, 0);
            }
        }
    }
    if (min_cost == INF)
    {
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            if (Flag_Item_Activate[ii] == true && !check_reservation(index, ii))
            {
                if (min_cost > getCost(index, ii, 0))
                {
                    min_taskID = ii;
                    min_cost = getCost(index, ii, 0);
                }
            }
        }
    }
    vector<coordinate> tmp = getPath(index, min_taskID, 0);
    robotList[index].assignTask(min_taskID, tmp, itemCoord);
    robotList[index].pathIndex = 0;
    robotList[index].AllocTask.taskId = min_taskID;
    robotList[index].AllocTask.taskcoord = itemCoord[min_taskID];
}
///////////////////////////////////////////////////////////////////

int main()
{
    //original code start
    srand((unsigned int)time(NULL));
    
    char data;
    
    FILE  * fp;
    
    fp = fopen("hwall10.txt", "r");
    
    int i = 0;
    int j = 0;
    
    while (fscanf(fp, "%c", &data) != EOF)
    {
        if (data == ' ')
        {
            j++;
        }
        else if (data == '\n')
        {
            i++;
            
            j = 0;
        }
        else if (data != 13)
        {
            hWallMatrix[i][j] = data - 48;
            
        }
    }
    
    
    fclose(fp);
    fp = fopen("vwall10.txt", "r");
    
    i = 0;
    j = 0;
    while (fscanf(fp, "%c", &data) != EOF)
    {
        if (data == ' ')
        {
            j++;
        }
        else if (data == '\n')
        {
            
            i++;
            
            j = 0;
        }
        else if (data != 13)
        {
            vWallMatrix[i][j] = data - 48;
            
        }
    }
    fclose(fp);
    
    int terreinMatrix[MAP_SIZE][MAP_SIZE];
    // map cost
    printf("print map\n\n");
    for (int ii = 0; ii < MAP_SIZE; ii++)
    {
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            terreinMatrix[ii][jj] = rand() % 200;
        }
    }
    
    int asdf;
    
    asdf = rand() % 40 + 50;
    
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        for (int ii = 0; ii < MAP_SIZE; ii++)
        {
            for (int jj = 0; jj < MAP_SIZE; jj++)
            {
                if (index % 2 == 0)
                {
                    robotList[index].travelCost[ii][jj] = terreinMatrix[ii][jj];
                }
                else
                {
                    robotList[index].travelCost[ii][jj] = asdf;
                }
            }
        }
    }
    
    //spawn tasks
    for (int ii = 0; ii< NUM_TASK; ii++)
    {
        itemCoord[ii].x = 11;
        itemCoord[ii].y = 11;
    }
    
    
    for (int ii = 0; ii < MAP_SIZE; ii++)
    {
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            
            printf("%4d ", robotList[0].travelCost[ii][jj]);
            //fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
            if (jj < MAP_SIZE - 1)
            {
                if (vWallMatrix[ii][jj] == 1)
                {
                    printf("| ");
                    //fprintf(fp, "| ");
                }
                else
                {
                    printf("  ");
                    //fprintf(fp, "  ");
                }
            }
        }
        printf("\n");
        //fprintf(fp, "\n");
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            if (ii < MAP_SIZE - 1)
            {
                if (hWallMatrix[ii][jj] == 1)
                {
                    printf("%4s ", "---");
                    //fprintf(fp, "%4s ","---");
                }
                else
                {
                    printf("%4c ", ' ');
                    //fprintf(fp, "%4c ",' ');
                }
                if (jj < MAP_SIZE - 1)
                {
                    printf("o ");
                    //fprintf(fp, "o ");
                }
            }
        }
        printf("\n");
        //    fprintf(fp, "\n");
    }
    printf("\n\n");
    //fprintf(fp,"\n");
    
    for (int ii = 0; ii < MAP_SIZE; ii++)
    {
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            
            printf("%4d ", robotList[1].travelCost[ii][jj]);
            //fprintf(fp, "%4d ", robotList[0].travelCost[ii][jj]);
            if (jj < MAP_SIZE - 1)
            {
                if (vWallMatrix[ii][jj] == 1)
                {
                    printf("| ");
                    //fprintf(fp, "| ");
                }
                else
                {
                    printf("  ");
                    //fprintf(fp, "  ");
                }
            }
        }
        printf("\n");
        //fprintf(fp, "\n");
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            if (ii < MAP_SIZE - 1)
            {
                if (hWallMatrix[ii][jj] == 1)
                {
                    printf("%4s ", "---");
                    //fprintf(fp, "%4s ","---");
                }
                else
                {
                    printf("%4c ", ' ');
                    //fprintf(fp, "%4c ",' ');
                }
                if (jj < MAP_SIZE - 1)
                {
                    printf("o ");
                    //fprintf(fp, "o ");
                }
            }
        }
        printf("\n");
        //    fprintf(fp, "\n");
    }
    printf("\n");
    
    
    
    int same;
    
    coordinate newItem;
    
    int doneList[NUM_TASK] = { 0, };
    
    
    // robot location
    for (int ii = 0; ii < NUM_ROBOT; ii++)
    {
        same = 1;
        while (same == 1)
        {
            newItem.x = rand() % MAP_SIZE;
            
            newItem.y = rand() % MAP_SIZE;
            
            same = 0;
            
            for (int jj = 0; jj < ii; jj++)
            {
                if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
                {
                    same = 1;
                }
            }
        }
        
        robotList[ii].Index_Robot = ii;
        robotList[ii].robotcoord.x = newItem.x;
        robotList[ii].robotcoord.y = newItem.y;
        
        
        printf("move robot %d to (%d, %d)\n", ii, newItem.x, newItem.y);
    }
    
    
    
    // task location
    for (int ii = 0; ii < NUM_TASK / 2; ii++)
    {
        same = 1;
        while (same == 1)
        {
            newItem.x = rand() % MAP_SIZE;
            
            newItem.y = rand() % MAP_SIZE;
            
            same = 0;
            
            for (int jj = 0; jj < NUM_ROBOT; jj++)
            {
                if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
                {
                    same = 1;
                }
            }
            for (int jj = 0; jj < ii; jj++)
            {
                if (newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
                {
                    same = 1;
                }
            }
        }
        
        
        itemCoord[ii].x = newItem.x;
        itemCoord[ii].y = newItem.y;
        
        printf("move item %d to (%d, %d)\n", ii, newItem.x, newItem.y);
    }
    
    
    // task cost
    int tempCost[2][NUM_TASK] = { 0, };
    for (int ii = 0; ii < NUM_TASK; ii++)
    {
        tempCost[0][ii] = rand() % 200;
        tempCost[1][ii] = rand() % 280;
    }
    
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            robotList[index].taskCost[ii] = tempCost[index % 2][ii];
        }
    }
    printf("\n");
    
    
    printf("task cost\ntask\t");
    
    for (int ii = 0; ii < NUM_TASK; ii++)
    {
        printf("%d\t", ii);
    }
    printf("\n");
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        printf("r%d :\t", index);
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            printf("%d\t", robotList[index].taskCost[ii]);
        }
        printf("\n");
    }
    printf("\n");
    
    
    int taskProgress[NUM_ROBOT] = { 0, };
    int movingProgress[NUM_ROBOT] = { 0, };
    
    int reach[NUM_ROBOT] = { 0, };
    
    int exitCondition = 0;
    int count = 0;
    int time = 0;
    
    task_produced = NUM_TASK / 2;
    int time_produced = TIME_MAX / 4;
    
    //original code end
    
    //initialize DStar_Lite Algorithm
    
    //initialize Task-Task DStar
    //Task_to_Task[0] : for Type1 Robot (Random Cost)
    //Task_to_Task[1] : for Type2 Robot (Uniform Cost)
    Task_to_Task[0] = new Dstar();
    Task_to_Task[1] = new Dstar();
    Task_to_Task[0]->init(0, 0, 1, 1);
    Task_to_Task[1]->init(0, 0, 1, 1);
    
    //initialize Robot-Task DStar
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        Robot *temp_robot = &robotList[index];
        for (int task_index = 0; task_index < task_produced; task_index++)
        {
            temp_robot->Robot_DStar[task_index] = new Dstar();
            //set start and goal
            temp_robot->Robot_DStar[task_index]->init(temp_robot->robotcoord.x, temp_robot->robotcoord.y, itemCoord[task_index].x, itemCoord[task_index].y);
            
            //기본값은 비활성화 상태이므로, 활성화 상태를 만들어줍니다.
            //비활성화 상태이면 replan method를 실행하지 않습니다.
            //replan method는 자동으로 돌아가게 해두었으므로, 실제로 호출할 일은 없습니다.
            Flag_Item_Activate[task_index] = true;
        }
        for (int task_index = task_produced; task_index < NUM_TASK; task_index++)
        {
            temp_robot->Robot_DStar[task_index] = new Dstar();
            //시작점과 끝점 설정
            temp_robot->Robot_DStar[task_index]->init(temp_robot->robotcoord.x, temp_robot->robotcoord.y, 1, 1);
        }
        
    }
    //Cost Initialization
    Robot *Type1_Robot = &robotList[0];
    Robot *Type2_Robot = &robotList[1];
    for (int ii = 0; ii < MAP_SIZE; ii++)
    {
        for (int jj = 0; jj < MAP_SIZE; jj++)
        {
            double temp_val[2] = {
                (double)Type1_Robot->getTravelCost(ii, jj),
                (double)Type2_Robot->getTravelCost(ii, jj)
            };
            //0 : up, 1 : down, 2 : left, 3 : right
            double cost[2][4] = {
                temp_val[0], temp_val[0], temp_val[0], temp_val[0],
                temp_val[1], temp_val[1], temp_val[1], temp_val[1]
            };
            
            for (int Type_i = 0; Type_i < 2; Type_i++)
            {
                if (!ii) cost[Type_i][2] = -1;
                if (ii == MAP_SIZE - 1) cost[Type_i][3] = -1;
                if (!jj) cost[Type_i][0] = -1;
                if (jj == MAP_SIZE - 1) cost[Type_i][1] = -1;
            }
            
            //update Robot-Task Cost
            for (int Robot_Index = 0; Robot_Index < NUM_ROBOT; Robot_Index++)
            {
                for (int Task_Index = 0; Task_Index < NUM_TASK; Task_Index++)
                {
                    if (Robot_Index % 2)
                        robotList[Robot_Index].Robot_DStar[Task_Index]->updateCell(ii, jj, cost[1]);
                    else
                        robotList[Robot_Index].Robot_DStar[Task_Index]->updateCell(ii, jj, cost[0]);
#ifdef DLite
                    robotList[Robot_Index].Robot_DStar[Task_Index]->replan();
#else
                    robotList[Robot_Index].Robot_DStar[Task_Index]->Dijkstra();
#endif
                }
            }
            
            //update Task-Task Cost
            Task_to_Task[0]->updateCell(ii, jj, cost[0]);
            Task_to_Task[1]->updateCell(ii, jj, cost[1]);
        }
    }
    
    //초기화 완료
    
    //Minimum Spanning Tree Draw
    draw_MSTree();
    task_alloc();
    
    int present_task = NUM_TASK / 2;
    int finished_task = 0;
    int remain_task = NUM_TASK / 2;
    
    // simulation
    while (time < TIME_MAX && exitCondition == 0)
    {
        // spawn new task
        if (time == time_produced && task_produced < NUM_TASK)
        {
            printf("at time %d : ", time);
            time_produced += TIME_MAX / NUM_TASK / 2;
            same = 1;
            while (same == 1)
            {
                newItem.x = rand() % MAP_SIZE;
                
                newItem.y = rand() % MAP_SIZE;
                
                same = 0;
                
                for (int jj = 0; jj < NUM_ROBOT; jj++)
                {
                    if (newItem.x == robotList[jj].robotcoord.x && newItem.y == robotList[jj].robotcoord.y)
                    {
                        same = 1;
                    }
                }
                for (int jj = 0; jj < task_produced; jj++)
                {
                    if (newItem.x == itemCoord[jj].x && newItem.y == itemCoord[jj].y)
                    {
                        same = 1;
                    }
                }
            }
            
            
            itemCoord[task_produced].x = newItem.x;
            itemCoord[task_produced].y = newItem.y;
            Flag_Item_Activate[task_produced] = true;
            
            printf("move item %d to (%d, %d)\n", task_produced, newItem.x, newItem.y);
            
            task_produced++;
            
            present_task++;
            remain_task--;
            
            draw_MSTree();
            task_alloc();
        }
        
        //simulate robot behavior
        for (int index = 0; index < NUM_ROBOT; index++)
        {
            if (robotList[index].status == MOVING)
            {
                
                if (movingProgress[index] == 0)
                {
                    movingProgress[index] += robotList[index].getTravelCost();
                    printf("robot %d is on (%d,%d) to move\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y);
                }
                if (movingProgress[index] > 0)
                {
                    //robotList[index].energy -= robotList[index].getTravelCost();
                    //movingProgress[index] -= robotList[index].getTravelCost();
                    
                    movingProgress[index] -= 10;
                    
                    printf("robot %d is moving in (%d,%d) --- %d\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y, movingProgress[index]);
                }
                else
                {
                    robotList[index].energy -= robotList[index].getTravelCost();
                }
                
                if (reach[index] == 1)
                {
                    robotList[index].status = WORKING;
                    robotList[index].pathIndex = 0;
                    movingProgress[index] = 0;
                    reach[index] = 0;
                    printf("robot %d arrive at task %d (%d,%d)\n", index, robotList[index].AllocTask.taskId,
                           robotList[index].AllocTask.taskcoord.x, robotList[index].AllocTask.taskcoord.y);
                }
                else if (movingProgress[index] <= 0)
                {
                    movingProgress[index] = 0;
                    printf("robot %d's travel finish in (%d,%d)\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y);
                    
                    if (!robotList[index].updatePostion())
                    {
                        alloc_new_task(index);
                    }
                    
                    if (robotList[index].atTask() == true)
                    {
                        reach[index] = 1;
                    }
                }
            }
            else if (robotList[index].status == WORKING)
            {
                if (taskProgress[index] == 0)
                {
                    taskProgress[index] += robotList[index].getTaskCost();
                    Flag_Item_Activate[robotList[index].AllocTask.taskId] = false;
                    printf("robot %d is on (%d,%d) to work\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y);
                }
                
                if (taskProgress[index] > 0)
                {
                    
                    taskProgress[index] -= 10;
                    
                    printf("robot %d is working on task %d\n", index, robotList[index].AllocTask.taskId);
                    
                }
                else if (taskProgress[index] <= 0)
                {
                    taskProgress[index] = 0;
                    
                    printf("robot %d finished task %d\n", index, robotList[index].AllocTask.taskId);
                    robotList[index].energy -= robotList[index].getTaskCost();
                    
                    //doneList[robotList[index].taskList[robotList[index].curr_task].taskId] = 1;
                    doneList[robotList[index].AllocTask.taskId] = 1;
                    
                    //robotList[index].curr_task++;
                    robotList[index].finTask[robotList[index].AllocTask.taskId] = true;
                    robotList[index].status = IDLE;
                    
                    taskProgress[index] = 0;
                    
                    draw_MSTree();
                    alloc_new_task(index);
                    present_task--;
                    finished_task++;
                }
                
            }
            else if (robotList[index].status == IDLE)
            {
                printf("robot %d is idle\n", index);
                
                /*if (robotList[index].curr_task < NUM_TASK)
                 {
                 if (robotList[index].AllocTask.taskId > -1)
                 {
                 robotList[index].status = MOVING;
                 }
                 }*/
                if (robotList[index].AllocTask.taskId > -1)
                {
                    robotList[index].status = MOVING;
                }
            }
        }
        time++;
        printf("\n===================================================================================\n");
        printf("number of task : %d / finished task : %d / remain task : %d", present_task, finished_task, remain_task);
        printf("\n===================================================================================\n\n");
        //system("pause");
        //simulate robot behavior end
        
        
        // end condition
        count = 0;
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            if (doneList[ii] == 1)
            {
                count++;
            }
            
            
        }
        if (count == NUM_TASK)
        {
            printf("finished task at time %d\n", time);
            exitCondition = 1;
        }
        
    }
    
    
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        robotList[index].calcCost();
        
        printf("robot %d remaining energy %d\n", index, robotList[index].energy);
    }
    
    
    printf("done list : ");
    for (int ii = 0; ii < NUM_TASK; ii++)
    {
        printf("%d ", doneList[ii]);
        
    }
    printf("\n");
    
    
    print_result(robotList, 0);
    
    delete Task_to_Task[0];
    delete Task_to_Task[1];
    for (int robo_index = 0; robo_index < NUM_ROBOT; robo_index++)
    {
        for (int task_index = 0; task_index < NUM_TASK; task_index++)
            delete robotList[robo_index].Robot_DStar[task_index];
    }
}
