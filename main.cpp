#define _CRT_SECURE_NO_WARNINGS    // 'fopen', 'fscanf' error handle

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "User_Defined_Function.h"
#include <algorithm>
#include "MST.h"
#include "Task_Allocation.h"
#include "time.h"
#include "FloodFill.h"

using namespace std;

int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int Meet_hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int Meet_vWallMatrix[MAP_SIZE][MAP_SIZE - 1];

Robot robotList[NUM_ROBOT];
coordinate itemCoord[NUM_TASK];
bool Flag_Item_Activate[NUM_TASK] = { false, };
Dstar * Task_to_Task[2];
int task_produced = 0;

vector<edge> MSTree[2];                        // 각각 uniform 맵으로부터 만들어진 mst, random 맵으로부터 만들어진 mst
// MSTree[0] : Type1, MSTree[1] : Type2
// 0&2번 로봇이 random => 0번 맵, 1&3번 로봇이 uniform => 1번 맵

// 초기화에만 false 입력, 2개 맵으로 부터의 mst는 동시에 업뎃

void print_result(Robot  * robotList, int mode);


/***************************************************************
 
 
 
 
                        MAIN START
 
 
 
 
 **************************************************************/

int main(int argc, char *argv[])
{
    //original code start
    srand((unsigned int)time(NULL));
    
    char config[10];
    char output_filename[100];
    strcpy(config, argv[1]);
    strcpy(output_filename, argv[2]);
    FILE *fp;
    if ((fp = fopen(output_filename, "r")))
    {
        fclose(fp);
    }
    else
    {
        fp = fopen(output_filename, "w");
        fprintf(fp, "E_R1,B_R1,E_R2,B_R2,E_R3,B_R3,E_R4,B_R4,E_total,B_Total,RealTime,SimTime,Task_Finish,Task_Total\n");
        fclose(fp);
    }
    int level = -1;
    
    if(config[0] == 69 || config[0] == 72 || config[0] == 78)
    {
        //input 'E' or 'N' or 'H'
        switch (config[0])
        {
            case 72:
                level = 5; // hard
            case 78:
                level = 3; // normal
                break;
            case 69:
                level = 1; // easy
                break;
        }
    }
    else
    {
        printf("ERROR : input right Map Level\n\n");
        exit(9);
    }
    ff floodflip(MAP_SIZE, MAP_SIZE);
    int map_block;
    bool exit_condition = false;
    while (!exit_condition)
    {
        
        for (int ii = 0; ii < MAP_SIZE; ii++)
            for (int jj = 0; jj < MAP_SIZE - 1; jj++)
            {
                int tmp = rand() % 10;
                if (tmp < level)
                    vWallMatrix[ii][jj] = 1;
            }
        for (int ii = 0; ii < MAP_SIZE - 1; ii++)
            for (int jj = 0; jj < MAP_SIZE; jj++)
            {
                int tmp = rand() % 10;
                if (tmp < level)
                    hWallMatrix[ii][jj] = 1;
            }
        
        map_block = floodflip.check_map();
        //floodflip.print_map(V_map, H_map);
        //cout << "Number of Block = " << map_block << endl;
        
        while (map_block != 1)
        {
            floodflip.edit_map(level);
            map_block = floodflip.check_map();
            //floodflip.print_map(V_map, H_map);
            //cout << "Number of Block = " << map_block << endl;
        }
        if(map_block == 1)
            exit_condition = true;
    }
    
    /*
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
     */
    
    int terreinMatrix[MAP_SIZE][MAP_SIZE];
    // map cost
#ifdef IS_PRINT
    printf("print map\n\n");
#endif
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
    
#ifdef IS_PRINT
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
#endif
    
    
    
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
        
#ifdef IS_PRINT
        printf("move robot %d to (%d, %d)\n", ii, newItem.x, newItem.y);
#endif
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
        
#ifdef IS_PRINT
        printf("move item %d to (%d, %d)\n", ii, newItem.x, newItem.y);
#endif
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
#ifdef IS_PRINT
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
#endif
    
    
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
    
    //TIME START
    time_t startTime = clock();
    
    
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
                if (ii == 0) cost[Type_i][2] = -1;
                if (ii == MAP_SIZE - 1) cost[Type_i][3] = -1;
                if (jj == 0) cost[Type_i][0] = -1;
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
                }
            }
            
            //update Task-Task Cost
            Task_to_Task[0]->updateCell(ii, jj, cost[0]);
            Task_to_Task[1]->updateCell(ii, jj, cost[1]);
        }
    }
    
    //first planning
    for (int Robot_Index = 0; Robot_Index < NUM_ROBOT; Robot_Index++)
    {
        for (int Task_Index = 0; Task_Index < task_produced; Task_Index++)
        {
#ifdef DLite
            robotList[Robot_Index].Robot_DStar[Task_Index]->replan();
#else
            robotList[Robot_Index].Robot_DStar[Task_Index]->Dijkstra();
#endif
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
#ifdef IS_PRINT
            printf("at time %d : ", time);
#endif
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
                for (int jj =  0; jj < task_produced; jj++)
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
            
            for(int Robot_Index = 0; Robot_Index < NUM_ROBOT; Robot_Index++)
                Change_Goal(Robot_Index, task_produced, newItem.x, newItem.y, 0);
#ifdef IS_PRINT
            printf("move item %d to (%d, %d)\n", task_produced, newItem.x, newItem.y);
#endif
            task_produced++;
            
            present_task++;
            remain_task++;
            
            draw_MSTree();
            task_alloc();
        }
        
        //simulate robot behavior
        for (int index = 0; index < NUM_ROBOT; index++)
        {
            if (robotList[index].status == MOVING)
            {
                
                if (movingProgress[index] == 0 && reach[index] != 1)
                {
                    if(robotList[index].energy >= robotList[index].getTravelCost())
                    {
                        movingProgress[index] += robotList[index].getTravelCost();
#ifdef IS_PRINT
                        printf("robot %d is on (%d,%d) to move\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y);
#endif
                        robotList[index].totalCost += robotList[index].getTravelCost();
                        robotList[index].totalBlocks ++;
                    }
                    else
                    {
#ifdef IS_PRINT
                        printf("robot %d has less energy : STOP\n", index);
#endif
                        robotList[index].status = STOP;
                        robotList[index].AllocTask.taskId = -1;
                        continue;
                    }
                }
                if (movingProgress[index] > 0)
                {
                    
                    if(movingProgress[index] >= 10) robotList[index].energy -= 10;
                    else robotList[index].energy -= movingProgress[index];
                    movingProgress[index] -= 10;
                    //printf("robot %d is moving in (%d,%d) --- %d\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y, movingProgress[index]);
                }
                
                if (reach[index] == 1)
                {
                    robotList[index].status = WORKING;
                    robotList[index].pathIndex = 0;
                    movingProgress[index] = 0;
                    reach[index] = 0;
#ifdef IS_PRINT
                    printf("robot %d arrive at task %d (%d,%d)\n", index, robotList[index].AllocTask.taskId,
                           robotList[index].AllocTask.taskcoord.x, robotList[index].AllocTask.taskcoord.y);
#endif
                }
                else if (movingProgress[index] <= 0)
                {
                    movingProgress[index] = 0;
                    //printf("robot %d's travel finish in (%d,%d)\n", index, robotList[index].robotcoord.x, robotList[index].robotcoord.y);
                    
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
                    if(robotList[index].energy >= robotList[index].getTaskCost())
                    {
                        taskProgress[index] += robotList[index].getTaskCost();
                        Flag_Item_Activate[robotList[index].AllocTask.taskId] = false;
#ifdef IS_PRINT
                        printf("robot %d is on (%d,%d) to work\n", index,
                               robotList[index].robotcoord.x, robotList[index].robotcoord.y);
#endif
                    }
                    else
                    {
#ifdef IS_PRINT
                        printf("robot %d has less energy : STOP\n", index);
#endif
                        robotList[index].status = STOP;
                        robotList[index].AllocTask.taskId = -1;
                        continue;
                    }
                }
                
                if (taskProgress[index] > 0)
                {
                    
                    if(taskProgress[index] >= 10) robotList[index].energy -= 10;
                    else robotList[index].energy -= taskProgress[index];
                    taskProgress[index] -= 10;
                    //printf("robot %d is working on task %d\n", index, robotList[index].AllocTask.taskId);
                    
                }
                if (taskProgress[index] <= 0)
                {
                    taskProgress[index] = 0;
                    robotList[index].totalCost += robotList[index].getTaskCost();
                    
#ifdef IS_PRINT
                    printf("robot %d finished task %d\n", index, robotList[index].AllocTask.taskId);
#endif
                    
                    doneList[robotList[index].AllocTask.taskId] = 1;
                    
                    robotList[index].status = IDLE;
                    
                    taskProgress[index] = 0;
                    
                    draw_MSTree();
                    //alloc_new_task(index);
                    task_alloc();
                    present_task--;
                    finished_task++;
                    
                }
                
            }
            //if state is IDLE
            else if (robotList[index].status == IDLE)
            {
#ifdef IS_PRINT
                printf("robot %d is idle\n", index);
#endif
                
                if (robotList[index].AllocTask.taskId > -1)
                {
                    robotList[index].status = MOVING;
                }
            }
        }
        time++;
        //printf("\n===================================================================================\n");
        //printf("number of task : %d / finished task : %d / remain task : %d", present_task, finished_task, remain_task);
        //printf("\n===================================================================================\n\n");
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
#ifdef IS_PRINT
            printf("finished task at time %d\n", time);
#endif
            exitCondition = 1;
        }
        
    }
    
#ifdef IS_PRINT
    printf("\n===================================================================================\n");
    printf("number of task : %d / finished task : %d / remain task : %d", task_produced, finished_task, present_task);
    printf("\n===================================================================================\n\n");
    
    for (int index = 0; index < NUM_ROBOT; index++)
    {
        printf("robot %d remaining energy %d\n", index, robotList[index].energy);
    }
    
    
    printf("done list : ");
    for (int ii = 0; ii < NUM_TASK; ii++)
    {
        printf("%d ", doneList[ii]);
        
    }
    printf("\n");
#endif
    
    //file print
    time_t endTime = clock();
    int Total_Consumption = 0, Total_Blocks = 0;
    fp = fopen(output_filename, "a");
    for(int robo_index=0; robo_index<NUM_ROBOT; robo_index++)
    {
        Total_Consumption += robotList[robo_index].totalCost;
        Total_Blocks += robotList[robo_index].totalBlocks;
        fprintf(fp, "%d,%d,",robotList[robo_index].totalCost, robotList[robo_index].totalBlocks);
    }
    fprintf(fp, "%d,%d,%.6lf,%d,%d,%d\n",Total_Consumption,Total_Blocks,(float)(endTime-startTime)/(CLOCKS_PER_SEC), time, finished_task, task_produced);
    fclose(fp);
    
#ifdef IS_PRINT
    print_result(robotList, 0);
#endif
    
    delete Task_to_Task[0];
    delete Task_to_Task[1];
    for (int robo_index = 0; robo_index < NUM_ROBOT; robo_index++)
    {
        for (int task_index = 0; task_index < NUM_TASK; task_index++)
            delete robotList[robo_index].Robot_DStar[task_index];
    }
}
/***************************************************************
 
 
 
 
                         MAIN END
 
 
 
 
 **************************************************************/

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
