#pragma once

#include <stdio.h>
#include <vector>
#include "Dstar.h"

#define NUM_ROBOT 4
#define NUM_TASK 16
#define MAX_ENERGY 2000
#define TIME_MAX 400

#define IDLE 0
#define WORKING 1
#define MOVING 2
#define STOP 3

#define UP 'w'
#define DOWN 's'
#define LEFT 'a'
#define RIGHT 'd'

#define DLite

#define INF 999999

using namespace std;

typedef struct alloc_task
{
    int taskID;
    double dis = INF;
}alloc;

class coordinate
{
public:
    int x;
    int y;
    
    coordinate(int xx, int yy)
    {
        x = xx;
        y = yy;
    }
    
    coordinate()
    {
        x = 0;
        y = 0;
    }
    
    bool operator == (const coordinate &s2) const {
        return ((x == s2.x) && (y == s2.y));
    }
    
    bool operator != (const coordinate &s2) const {
        return ((x != s2.x) || (y != s2.y));
    }
    
    //~coordinate();
};

class task
{
public:
    coordinate taskcoord;
    
    std::vector<coordinate> path;
    
    int taskId;
    
    int taskCost;
    
};

class Robot
{
public:
    Dstar * Robot_DStar[NUM_TASK];
    
    vector<coordinate> pre_path;
    
    coordinate robotcoord;
    
    coordinate taskCoord[NUM_TASK];
    
    int travelCost[MAP_SIZE][MAP_SIZE]; //travel cost of block coordinate
    
    int taskCost[NUM_TASK]; // cost of a task performed by this robot
    
    //task taskList[NUM_TASK];// list of tasks assgined to this robot
    
    task AllocTask;
    
    //int alloc_taskID;
    
    int totalCost;// total energy consumed
    int totalBlocks; // total number of blocks traveled
    
    //int num_task;
    int status;
    int energy;
    
    //int curr_task;
    int pathIndex;
    int Index_Robot;
    
    Robot()
    {
        robotcoord.x = 0;
        robotcoord.y = 0;
        
        //num_task = 0;
        
        energy = MAX_ENERGY;
        
        status = IDLE;
        //curr_task = 0;
        pathIndex = 0;
        
        for (int ii = 0; ii < NUM_TASK; ii++)
        {
            taskCoord[ii].x = 0;
            taskCoord[ii].y = 0;
            taskCost[ii] = 0;
        }
        
        AllocTask.taskId = -1;
        
        for (int ii = 0; ii < MAP_SIZE; ii++)
        {
            for (int jj = 0; jj < MAP_SIZE; jj++)
            {
                travelCost[ii][jj] = 0;
            }
        }
        
        totalCost = 0;
        totalBlocks = 0;
    }
    
    void assignTask(int taskId, std::vector<coordinate> inputPath, coordinate location[]);
    
    int getTravelCost();
    int getTravelCost(int x, int y);
    
    int getTaskCost();
    int getTaskCost(int x);
    
    coordinate getCurrentPosition();
    
    bool updatePostion();
    
    bool atTask();
    
};
