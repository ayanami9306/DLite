#include "RobotClass.h"
#include "User_Defined_Function.h"

extern Robot robotList[NUM_ROBOT];
extern int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
extern int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
extern int Meet_hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
extern int Meet_vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
extern bool Flag_Item_Activate[NUM_TASK];

void Robot::assignTask(int taskId, std::vector<coordinate> inputPath, coordinate itemList[])
{
    
    coordinate current;
    coordinate next;
    
    
    //if (num_task < NUM_TASK)
    if (taskId < NUM_TASK)
    {
        
        current.x = robotcoord.x;
        current.y = robotcoord.y;
        next.x = itemList[taskId].x;
        next.y = itemList[taskId].y;
        
#ifdef IS_PRINT
        printf("path assigned from Robot %d(%d, %d) to Item %d(%d, %d)\n", Index_Robot,current.x, current.y, taskId, next.x, next.y);
#endif
        AllocTask.taskId = taskId;
        AllocTask.path = inputPath;
        AllocTask.taskcoord.x = next.x;
        AllocTask.taskcoord.y = next.y;
        //num_task++;
        
    }
    else
    {
#ifdef IS_PRINT
        printf("ERROR : taskID is invalid");
#endif
    }
}

int Robot::getTravelCost()
{
    return travelCost[robotcoord.y][robotcoord.x];
}

int Robot::getTravelCost(int x, int y)
{
    return travelCost[y][x];
}

int Robot::getTaskCost()
{
    //return taskCost[taskList[curr_task].taskId];
    return taskCost[AllocTask.taskId];
}

int Robot::getTaskCost(int x)
{
    return taskCost[x];
}

coordinate Robot::getCurrentPosition()
{
    return robotcoord;
}

bool Robot::atTask()
{
    //if (robotcoord.x == taskList[curr_task].taskcoord.x && robotcoord.y == taskList[curr_task].taskcoord.y)
    if (robotcoord.x == AllocTask.taskcoord.x && robotcoord.y == AllocTask.taskcoord.y && Flag_Item_Activate[AllocTask.taskId])
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Robot::updatePostion()
{
    
    //해당 지점에서 벽이 있는지 둘러보고, 있으면 업데이트 합니다.
    //이미 보았던 벽은 업데이트 하지 않게 되어있습니다.
    int ix = robotcoord.x, iy = robotcoord.y;
    bool is_Meet_Wall = false;
    
    if (iy < MAP_SIZE - 1 && ix < MAP_SIZE)
    {
        if (hWallMatrix[iy][ix] && !Meet_hWallMatrix[iy][ix])
        {
            is_Meet_Wall = true;
            BroadCast_Walls(ix, iy, ix, iy + 1);
            Meet_hWallMatrix[iy][ix] = 1;
        }
        if (iy - 1 >= 0)
        {
            if (hWallMatrix[iy - 1][ix] && !Meet_hWallMatrix[iy - 1][ix])
            {
                is_Meet_Wall = true;
                BroadCast_Walls(ix, iy - 1, ix, iy);
                Meet_hWallMatrix[iy - 1][ix] = 1;
            }
        }
    }
    
    if (ix < MAP_SIZE - 1 && iy < MAP_SIZE)
    {
        if (vWallMatrix[iy][ix] && !Meet_vWallMatrix[iy][ix])
        {
            is_Meet_Wall = true;
            BroadCast_Walls(ix, iy, ix + 1, iy);
            Meet_vWallMatrix[iy][ix] = 1;
        }
        if (ix - 1 >= 0)
        {
            if (vWallMatrix[iy][ix - 1] && !Meet_vWallMatrix[iy][ix - 1])
            {
                is_Meet_Wall = true;
                BroadCast_Walls(ix - 1, iy, ix, iy);
                Meet_vWallMatrix[iy][ix - 1] = 1;
            }
        }
    }
    
    //벽을 만나 업데이트 하는 경우가 생긴 경우
    //모든 로봇 - Task에 대해 경로를 업데이트합니다.
    if (is_Meet_Wall)
    {
        //if you meet wall, replan path about all task-robot DStar
        for (int Robot_Index = 0; Robot_Index < NUM_ROBOT; Robot_Index++)
            for (int Task_Index = 0; Task_Index < NUM_TASK; Task_Index++)
            {
                if (Flag_Item_Activate[Task_Index])
                {
#ifdef DLite
                    robotList[Robot_Index].Robot_DStar[Task_Index]->replan();
#else
                    robotList[Robot_Index].Robot_DStar[Task_Index]->Dijkstra();
#endif
                }
            }
    }
    
    //기존에 알고있었던 패스가 재구성된 패스랑 다를 경우 false를 리턴합니다
    
    //아래부분부터는 구현해주셔야할 내용입니다.
    //리턴값이 false일경우 1G이내의 비선점된 아이템을 서치하여 재할당합니다.
    //그런 아이템이 없을 경우 의논하였던 대로 그리디하게 할당합니다.
    //그리고 패스 업데이트를 다시 합니다.
    //아마도 pathIndex 값을 재수정하셔야할 것 같습니다. (패스가 달라지므로)
    //패스를 한번 리니어 서치해서 현재 위치와 같은 곳을 패스인덱스로 설정하시면 될 것 같습니다.
    //cost = 거리 코스트 + task cost임을 기억하시고 서치하시면 될것같습니다.
    //구현해주셔야할 내용 끝
    
    if (!is_path_same(pre_path, getPath(Index_Robot, AllocTask.taskId, 0)))
    {
        for (int item = 0; item < NUM_TASK; item++)
        {
            Change_Start(Index_Robot, item, robotcoord.x, robotcoord.y, 0);
        }
        return false;
    }
    else
    {
        //재구성된 패스가 이전 패스와 동일할 경우 위치를 다음 위치로 업데이트하고 true를 리턴합니다.
        //if original path is same, execute updateposition
        //return true
        pathIndex++;
        
        if (pathIndex < AllocTask.path.size())
        {
            robotcoord.x = AllocTask.path.at(pathIndex).x;
            robotcoord.y = AllocTask.path.at(pathIndex).y;
        }
        else
        {
            pathIndex = 0;
        }
        
        return true;
    }
}

