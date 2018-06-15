#define _CRT_SECURE_NO_WARNINGS	// 'fopen', 'fscanf' error handle

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <windows.h>
#include <vector>
#include "Dstar.h"

#define NUM_ROBOT 4
#define NUM_TASK 16
#define MAX_ENERGY 2000
#define TIME_MAX 400

#define IDLE 0
#define WORKING 1
#define MOVING 2

#define UP 'w'
#define DOWN 's'
#define LEFT 'a'
#define RIGHT 'd'

#define DLite

#define INF 999999
int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int Meet_hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
int Meet_vWallMatrix[MAP_SIZE][MAP_SIZE - 1];
int terreinMatrix[MAP_SIZE][MAP_SIZE];
int asdf = 0;

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
	int finTask[NUM_TASK] = { INF, };

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

			AllocTask.taskId = -1;

		}

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

	void calcCost();

	int getTravelCost();
	int getTravelCost(int x, int y);

	int getTaskCost();
	int getTaskCost(int x);

	coordinate getCurrentPosition();

	bool updatePostion();

	bool atTask();

};

//int pathValidation(std::vector <coordinate> inputPath, coordinate start, coordinate end)
//{
//	int pass = 0;
//
//
//
//	if (inputPath.at(0).x != start.x || inputPath.at(0).y != start.y)
//	{
//		pass = 1;
//	}
//
//
//	if (inputPath.at(inputPath.size() - 1).x != end.x || inputPath.at(inputPath.size() - 1).y != end.y)
//	{
//		pass = 2;
//	}
//
//
//	int xdiff = 0;
//	int ydiff = 0;
//
//	int prevx = 0;
//	int prevy = 0;
//	int nextx = 0;
//	int nexty = 0;
//
//	for (int ii = 0; ii < inputPath.size() - 1; ii++)
//	{
//		nextx = inputPath.at(ii + 1).x;
//		nexty = inputPath.at(ii + 1).y;
//		prevx = inputPath.at(ii).x;
//		prevy = inputPath.at(ii).y;
//
//		xdiff = nextx - prevx;
//		ydiff = nexty - prevy;
//
//		if (xdiff == -1 && ydiff == 0)
//		{
//			//left
//			if (vWallMatrix[prevy][prevx - 1] == 1)
//			{
//				pass = 4;
//			}
//		}
//		else if (xdiff == 1 && ydiff == 0)
//		{
//			//right
//			if (vWallMatrix[prevy][prevx] == 1)
//			{
//				pass = 4;
//			}
//		}
//		else if (ydiff == -1 && xdiff == 0)
//		{
//			//up
//			if (hWallMatrix[prevy - 1][prevx] == 1)
//			{
//				pass = 4;
//			}
//		}
//		else if (ydiff == 1 && xdiff == 0)
//		{
//			//down
//			if (hWallMatrix[prevy][prevx] == 1)
//			{
//				pass = 4;
//			}
//		}
//		else
//		{
//			pass = 8;
//		}
//	}
//
//	if (pass > 0)
//	{
//		printf("invalid path : ");
//
//		if (pass >= 8)
//		{
//			printf("jump path,");
//			pass -= 8;
//		}
//
//		if (pass >= 4)
//		{
//			printf("path through wall,");
//			pass -= 4;
//		}
//		if (pass >= 2)
//		{
//			printf("end position,");
//			pass -= 2;
//		}
//		if (pass == 1)
//		{
//			printf("start position");
//			pass -= 1;
//		}
//		printf("\n");
//
//		pass = 1;
//	}
//
//	return pass;
//
//}

Robot robotList[NUM_ROBOT];
coordinate itemCoord[NUM_TASK];
bool Flag_Item_Activate[NUM_TASK] = { false, };
Dstar * Task_to_Task[2];

void Robot::assignTask(int taskId, std::vector<coordinate> inputPath, coordinate itemList[])
{

	coordinate current;
	coordinate next;

	//int pass = 0;


	//if (num_task < NUM_TASK)
	if (taskId < NUM_TASK)
	{
		/*if (num_task == 0)
		{
			current.x = robotcoord.x;
			current.y = robotcoord.y;
		}
		else
		{
			current.x = itemList[taskList[num_task - 1].taskId].x;
			current.y = itemList[taskList[num_task - 1].taskId].y;
		}

		next.x = itemList[taskId].x;
		next.y = itemList[taskId].y;*/

		current.x = robotcoord.x;
		current.y = robotcoord.y;
		next.x = itemList[taskId].x;
		next.y = itemList[taskId].y;


		printf("path assigned from (%d, %d) to (%d, %d)\n", current.x, current.y, next.x, next.y);
		AllocTask.taskId = taskId;
		AllocTask.path = inputPath;
		AllocTask.taskcoord.x = next.x;
		AllocTask.taskcoord.y = next.y;
		//num_task++;

		/*pass = pathValidation(inputPath, current, next);

		if (pass == 0)
		{
			taskList[num_task].taskId = taskId;
			taskList[num_task].path = inputPath;
			taskList[num_task].taskcoord.x = next.x;
			taskList[num_task].taskcoord.y = next.y;
			num_task++;
		}
		else
		{
			printf("assignment validation failed\n");
		}*/


	}
	else
	{
		printf("too many task assignment failed\n");
	}
}

void Robot::calcCost()
{
	int tempCost = 0;
	int tempBlocks = 0;
	//int costList = 0;

	/*for (int ii = 0; ii < num_task; ii++)
	{
		tempCost = 0;
		tempBlocks = 0;

		if (taskList[ii].path.empty() == false)
		{
			for (int jj = 0; jj < taskList[ii].path.size(); jj++)
			{
				tempCost += travelCost[taskList[ii].path.at(jj).y][taskList[ii].path.at(jj).x];
				tempBlocks++;
			}
		}

		totalCost += tempCost;
		totalBlocks += tempBlocks;


		totalCost += taskCost[taskList[ii].taskId];

	}*/
	for (int ii = 0; ii < NUM_TASK; ii++)
	{
		tempCost = 0;
		tempBlocks = 0;
		if (AllocTask.path.empty() == false)
		{
			for (int jj = 0; jj < NUM_TASK; jj++)
			{
				if (finTask[jj] == true)
				{
					tempCost += travelCost[itemCoord[jj].x][itemCoord[jj].y];
					tempBlocks++;
					totalCost += taskCost[jj];
				}
			}
		}
		totalCost += tempCost;
		totalBlocks += tempBlocks;
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
	if (robotcoord.x == AllocTask.taskcoord.x && robotcoord.y == AllocTask.taskcoord.y)
	{
		return true;
	}
	else
	{
		return false;
	}
}

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

////////////////////////////////////////////////////////////////////
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
		list<state>::iterator iter;
		printf("Robot: %d(%d,%d), Task : %d(%d,%d)\n-------------\n", RobotIndex, temp_robot->robotcoord.x, temp_robot->robotcoord.y, ItemIndex, itemCoord[ItemIndex].x, itemCoord[ItemIndex].y);
		for (iter = mypath.begin(); iter != mypath.end(); iter++)
		{
			printf("%d, %d\n", iter->x, iter->y);
		}
		printf("-----------------------\n");
	}
	else
	{
		list<state>mypath;
#ifdef DLite
		mypath = Task_to_Task[RobotIndex]->getPath();
#else
		mypath = Task_to_Task[RobotIndex]->getDijkstraPath();
#endif
		list<state>::iterator iter;
		printf("-------------\n");
		for (iter = mypath.begin(); iter != mypath.end(); iter++)
		{
			printf("%d, %d\n", iter->x, iter->y);
		}
		printf("-----------------------\n");
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
#ifdef DLite
		robotList[RobotIndex].Robot_DStar[ItemIndex]->replan();
#else
		robotList[RobotIndex].Robot_DStar[ItemIndex]->Dijkstra();
#endif
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
#ifdef DLite
		robotList[RobotIndex].Robot_DStar[ItemIndex]->replan();
#else
		robotList[RobotIndex].Robot_DStar[ItemIndex]->Dijkstra();
#endif
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

bool Robot::updatePostion()
{
	 //pre_path = getPath(Index_Robot, AllocTask.taskId, 0);

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


		/*if (pathIndex < taskList[curr_task].path.size())
		{

			robotcoord.x = taskList[curr_task].path.at(pathIndex).x;
			robotcoord.y = taskList[curr_task].path.at(pathIndex).y;
		}
		else
		{
			pathIndex = 0;
		}*/

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

///////////////////////////////////////////////////////////////////

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

bool tree[NUM_TASK][NUM_TASK] = { false, };			// task들의 연결을 확인
std::vector<edge> MSTree[2];						// 각각 uniform 맵으로부터 만들어진 mst, random 맵으로부터 만들어진 mst
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
	{
		for (int j = 0; j < NUM_TASK; j++)
			tree[i][j] = false;
	}
}
// 초기화에만 false 입력, 2개 맵으로 부터의 mst는 동시에 업뎃
void draw_MSTree()
{
	for (int map_type = 0; map_type < 2; map_type++)
	{
		double task_graph[NUM_TASK][NUM_TASK] = { INF, };	// 각 task들 사이의 cost 값

		for (int ii = 0; ii < NUM_TASK; ii++)
		{
			if (Flag_Item_Activate[ii] == true)
			{ 
				Change_Start(map_type, 0, itemCoord[ii].x, itemCoord[ii].y, 1);
				for (int jj = 0; jj < NUM_TASK; jj++)
				{
					if (ii != jj && Flag_Item_Activate[jj] == true)
					{
						Change_Goal(map_type, 0, itemCoord[jj].x, itemCoord[jj].y, 1);
						task_graph[ii][jj] = getCost(map_type, 0, 1);
					}
					else
						task_graph[ii][jj] = INF;
				}
			}
			else
			{
				for (int jj = 0; jj < NUM_TASK; jj++)
					task_graph[ii][jj] = INF;
			}
		}

		/*printf("\ntask graph[%d]\n", map_type);
		for (int i = 0; i < NUM_TASK; i++)
		{
			for (int j = 0; j < NUM_TASK; j++)
			{
				if (task_graph[i][j] == INF)
					printf("INF\t");
				else
					printf("%.2f\t", task_graph[i][j]);
			}
			printf("\n");
		}*/

		int task_cnt = 0;
		for (int i = 0; i < NUM_TASK; i++)
		{
			if (Flag_Item_Activate[i] == true)
				task_cnt++;
		}
		
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
	srand(time(NULL));

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

	// map cost
	printf("print map\n\n");
	for (int ii = 0; ii < MAP_SIZE; ii++)
	{
		for (int jj = 0; jj < MAP_SIZE; jj++)
		{
			terreinMatrix[ii][jj] = rand() % 200;
		}
	}
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
		//	fprintf(fp, "\n");
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
		//	fprintf(fp, "\n");
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

	/*
	coordinate currentPos;
	std::vector <coordinate> asdff;

	currentPos.x = 0;
	currentPos.y = 0;

	asdff.push_back(currentPos);

	currentPos.x = 1;

	asdff.push_back(currentPos);

	currentPos.y = 1;

	asdff.push_back(currentPos);

	robotList[0].assignTask(0, asdff, itemCoord);
	*/
	

	int taskProgress[NUM_ROBOT] = { 0, };
	int movingProgress[NUM_ROBOT] = { 0, };

	int reach[NUM_ROBOT] = { 0, };

	int exitCondition = 0;
	int count = 0;
	int time = 0;

	int task_produced = NUM_TASK / 2;
	int time_produced = TIME_MAX / 4;

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
					if (Robot_Index % 2)
						robotList[Robot_Index].Robot_DStar[Task_Index]->updateCell(ii, jj, cost[1]);
					else
						robotList[Robot_Index].Robot_DStar[Task_Index]->updateCell(ii, jj, cost[0]);
			}

			//update Task-Task Cost
			Task_to_Task[0]->updateCell(ii, jj, cost[0]);
			Task_to_Task[1]->updateCell(ii, jj, cost[1]);
		}
	}

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
	for (int robo_index = 0; robo_index < NUM_ROBOT; robo_index)
	{
		for (int task_index = 0; task_index < NUM_TASK; task_index++)
			delete robotList[robo_index].Robot_DStar[task_index];
	}
}
