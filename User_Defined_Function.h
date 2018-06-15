#include "RobotClass.h"

void PrintPath(int RobotIndex, int ItemIndex, int code);
int getCost(int RobotIndex, int ItemIndex, int code);
void Change_Start(int RobotIndex, int ItemIndex, int x, int y, int code);
void Change_Goal(int RobotIndex, int ItemIndex, int x, int y, int code);
vector<coordinate> getPath(int RobotIndex, int ItemIndex, int code);
void BroadCast_Walls(int x1, int y1, int x2, int y2);
bool is_path_same(vector<coordinate> path1, vector<coordinate> path2);
