//
//  User_Defined_Function.cpp
//  DLite
//
//  Created by Nozomi on 2018. 6. 15..
//  Copyright © 2018년 JiHoon. All rights reserved.
//

#include <stdio.h>
#include "User_Defined_Function.h"

extern Robot robotList[NUM_ROBOT];
extern coordinate itemCoord[NUM_TASK];
extern Dstar * Task_to_Task[2];
extern bool Flag_Item_Activate[NUM_TASK];

void PrintPath(int RobotIndex, int ItemIndex, int code)
{
    if (!code)
    {
        Robot *temp_robot = &robotList[RobotIndex];
        list<state>mypath;
#ifdef DLite
        mypath = temp_robot->Robot_DStar[ItemIndex]->getPath();
#else
        mypath = temp_robot->Robot_DStar[ItemIndex]->getDijkstraPath();
#endif
#ifdef IS_PRINT
        list<state>::iterator iter;
        printf("Robot: %d(%d,%d), Task : %d(%d,%d)\n-------------\n", RobotIndex, temp_robot->robotcoord.x, temp_robot->robotcoord.y, ItemIndex, itemCoord[ItemIndex].x, itemCoord[ItemIndex].y);
        for (iter = mypath.begin(); iter != mypath.end(); iter++)
        {
            printf("%d, %d\n", iter->x, iter->y);
        }
        printf("-----------------------\n");
#endif
    }
    else
    {
        list<state>mypath;
#ifdef DLite
        mypath = Task_to_Task[RobotIndex]->getPath();
#else
        mypath = Task_to_Task[RobotIndex]->getDijkstraPath();
#endif
#ifdef IS_PRINT
        list<state>::iterator iter;
        printf("-------------\n");
        for (iter = mypath.begin(); iter != mypath.end(); iter++)
        {
            printf("%d, %d\n", iter->x, iter->y);
        }
        printf("-----------------------\n");
#endif
    }
}

int getCost(int RobotIndex, int ItemIndex, int code)
{
    if (!code)
    {
#ifdef DLite
        return robotList[RobotIndex].Robot_DStar[ItemIndex]->getCost();
#else
        return robotList[RobotIndex].Robot_DStar[ItemIndex]->getDijkstraCost();
#endif
    }
    else
    {
#ifdef DLite
        return Task_to_Task[RobotIndex]->getCost();
#else
        return Task_to_Task[RobotIndex]->getDijkstraCost();
#endif
    }
}

void Change_Start(int RobotIndex, int ItemIndex, int x, int y, int code)
{
    if (!code)
    {
        robotList[RobotIndex].Robot_DStar[ItemIndex]->updateStart(x, y);
        if(Flag_Item_Activate[ItemIndex])
        {
#ifdef DLite
            robotList[RobotIndex].Robot_DStar[ItemIndex]->replan();
#else
            robotList[RobotIndex].Robot_DStar[ItemIndex]->Dijkstra();
#endif
        }
    }
    else
    {
        Task_to_Task[RobotIndex]->updateStart(x, y);
#ifdef DLite
        Task_to_Task[RobotIndex]->replan();
#else
        Task_to_Task[RobotIndex]->Dijkstra();
#endif
    }
}

void Change_Goal(int RobotIndex, int ItemIndex, int x, int y, int code)
{
    if (!code)
    {
        robotList[RobotIndex].Robot_DStar[ItemIndex]->updateGoal(x, y);
        if(Flag_Item_Activate[ItemIndex])
        {
#ifdef DLite
            robotList[RobotIndex].Robot_DStar[ItemIndex]->replan();
#else
            robotList[RobotIndex].Robot_DStar[ItemIndex]->Dijkstra();
#endif
        }
    }
    else
    {
        Task_to_Task[RobotIndex]->updateGoal(x, y);
#ifdef DLite
        Task_to_Task[RobotIndex]->replan();
#else
        Task_to_Task[RobotIndex]->Dijkstra();
#endif
    }
}

vector<coordinate> getPath(int RobotIndex, int ItemIndex, int code)
{
    vector<coordinate> path;
    coordinate item_path;
    list<state>mypath;
    if (!code)
    {
        Robot *temp_robot = &robotList[RobotIndex];
#ifdef DLite
        mypath = temp_robot->Robot_DStar[ItemIndex]->getPath();
#else
        mypath = temp_robot->Robot_DStar[ItemIndex]->getDijkstraPath();
#endif
    }
    else
    {
#ifdef DLite
        mypath = Task_to_Task[RobotIndex]->getPath();
#else
        mypath = Task_to_Task[RobotIndex]->getDijkstraPath();
#endif
    }
    
    list<state>::iterator iter;
    for (iter = mypath.begin(); iter != mypath.end(); iter++)
    {
        item_path.x = iter->x;
        item_path.y = iter->y;
        path.push_back(item_path);
    }
    return path;
    
}

void BroadCast_Walls(int x1, int y1, int x2, int y2)
{
    
    for (int Robot_index = 0; Robot_index < NUM_ROBOT; Robot_index++)
    {
        Robot *temp_robot = &robotList[Robot_index];
        for (int Task_index = 0; Task_index < NUM_TASK; Task_index++)
        {
            temp_robot->Robot_DStar[Task_index]->updateWall(x1, y1, x2, y2);
        }
    }
    
    Task_to_Task[0]->updateWall(x1, y1, x2, y2);
    Task_to_Task[1]->updateWall(x1, y1, x2, y2);
}

bool is_path_same(vector<coordinate> path1, vector<coordinate> path2)
{
    if (path1.size() != path2.size()) return false;
    else
    {
        for (int i = 0; i<path1.size(); i++)
            if ((path1.at(i).x != path2.at(i).x) || (path1.at(i).y != path2.at(i).y))
                return false;
    }
    return true;
}
