#include "FloodFill.h"
#include <iostream>
#include <time.h>
#include <vector>
#include <algorithm>

#include "string.h"

extern int hWallMatrix[MAP_SIZE - 1][MAP_SIZE];
extern int vWallMatrix[MAP_SIZE][MAP_SIZE - 1];

using namespace std;

ff::ff(int row_size, int col_size)
{
	srand((unsigned)time(NULL));
	this->row = row_size;
	this->col = col_size;
	map = new int*[row];
	for (int i = 0; i < row; i++)
	{
		map[i] = new int[col];
		memset(map[i], 0, sizeof(int) * col);
	}
}
ff::~ff()
{
	for (int i = 0; i < row; i++)
		delete[] map[i];
	delete[] map;
}

void ff::clear_map()
{
	for (int i = 0; i < row; i++)
		memset(map[i], 0, sizeof(int) * col);
}
int ff::check_map()
{
	clear_map();
	num_block = 1;

	int x = rand() % col;
	int y = rand() % row;

	bool exit_condition = false;
	while (!exit_condition)
	{
		flood_loop(x, y, num_block);

		exit_condition = true;
		for (int i = 0; i < row && exit_condition; i++)
		{
			for (int j = 0; j < col; j++)
			{
				if(!map[i][j])
				{
					exit_condition = false;
					x = i;
					y = j;
					break;
				}	
			}
		}
		if (exit_condition == true)
			return num_block;
		else
			num_block++;
	}
    return -1;
}
void ff::flood_loop(int x, int y, int color)
{
	if (map[x][y] == color)
		return;

	map[x][y] = color;

	if (row > x + 1)
	{
		if (hWallMatrix[x][y] == false)
			flood_loop(x + 1, y, color);
	}
	if (col > y + 1)
	{
		if (vWallMatrix[x][y] == false)
			flood_loop(x, y + 1, color);
	}
	if (x - 1 > -1)
	{
		if (hWallMatrix[x - 1][y] == false)
			flood_loop(x - 1, y, color);
	}
	if (y - 1 > -1)
	{
		if (vWallMatrix[x][y - 1] == false)
			flood_loop(x, y - 1, color);
	}
}
void ff::print_map()
{
	cout << endl;
	cout << "<MAP>" << endl;
	for (int jj = 0; jj < col; jj++)
		cout << " ----";
	cout << endl;
	for (int ii = 0; ii < row; ii++)
	{
		cout << "|";
		for (int jj = 0; jj < col; jj++)
		{
			if (map[ii][jj] == 1)
				cout << "    ";
			else
			{
				cout.width(3);
				cout << map[ii][jj] << " ";
				cout.width();
			}
			if (jj < col - 1)
			{
				if (vWallMatrix[ii][jj] == 1)
					cout << "|";
				else
					cout << " ";
			}
		}
		cout << "|";
		cout << endl;
		cout << " ";
		for (int jj = 0; jj < col; jj++)
		{
			if (ii < row - 1)
			{
				if (hWallMatrix[ii][jj] == 1)
					cout << "----";
				else
					cout << "    ";
				if (jj < col - 1)
					cout << ".";
			}
			else
				cout << "---- ";
		}
		cout << endl << endl;
	}
}

typedef struct coordi
{
	int x;
	int y;
}coordi;
bool comp(std::pair<int, int> A, std::pair<int, int> B)
{
	return (A.second < B.second);
}
void ff::edit_map(int level)
{
	std::vector<std::pair<int, int>> order;
	for (int n = 1; n <= num_block; n++)
	{
		int cnt = 0;
		std::pair<int, int> tmp;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				if (map[i][j] == n)
					cnt++;
			}
		}
		tmp.first = n;
		tmp.second = cnt;
		order.push_back(tmp);
	}
	std::sort(order.begin(), order.end(), comp);

	bool exit_cond = false;
	for (int blkID = 0; blkID < num_block && !exit_cond; blkID++)
	{
		vector<coordi> tmp_vec;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				if (map[i][j] == order[blkID].first)
				{
					coordi tmp;
					tmp.x = i;
					tmp.y = j;
					tmp_vec.push_back(tmp); // 구분된 block 위치 정보 수집
				}
			}
		}
		// H:50%, N:66%, E:90% 확률로 벽 제거 대상에 포함
		for (int id = 0; id < tmp_vec.size(); id++)
		{
			int freq = rand() % 10;
			if (freq > level)
				tmp_vec.erase(tmp_vec.begin() + id);
		}
		// 각 제거 대상의 사방에 대해 벽이 존재할 시 20%확률로 벽 제거
		for (int id = 0; id < tmp_vec.size(); id++)
<<<<<<< HEAD
		{
            int new_coordinate[2];
=======
		{
			int remove;
>>>>>>> 3f52a0255d5685c5d67c0041cff32291fefce8b8
			coordi tmp = tmp_vec[id];
			if (tmp.x - 1 > -1)
			{
				if (map[tmp.x - 1][tmp.y] != order[blkID].first)
				{
					if (hWallMatrix[tmp.x - 1][tmp.y] == true) // 상
<<<<<<< HEAD
					{
                        new_coordinate[0] = rand()%MAP_SIZE;
                        new_coordinate[1] = rand()%(MAP_SIZE-1);
                        hWallMatrix[tmp.x - 1][tmp.y] = false;
                        while(hWallMatrix[new_coordinate[1]][new_coordinate[0]])
                        {
                            new_coordinate[0] = rand() % MAP_SIZE;
                            new_coordinate[1] = rand() % (MAP_SIZE-1);
                        }
                        hWallMatrix[new_coordinate[1]][new_coordinate[0]] = true;
=======
					{
						remove = rand() % 10;
						if (remove < remove_probab)
							hWallMatrix[tmp.x - 1][tmp.y] = false;
>>>>>>> 3f52a0255d5685c5d67c0041cff32291fefce8b8
					}
				}
			}
			if (row > tmp.x + 1)
			{
				if (map[tmp.x + 1][tmp.y] != order[blkID].first)
				{
					if (hWallMatrix[tmp.x][tmp.y] == true) // 하
<<<<<<< HEAD
					{
                        new_coordinate[0] = rand()%MAP_SIZE;
                        new_coordinate[1] = rand()%(MAP_SIZE-1);
                        hWallMatrix[tmp.x][tmp.y] = false;
                        while(hWallMatrix[new_coordinate[1]][new_coordinate[0]])
                        {
                            new_coordinate[0] = rand() % MAP_SIZE;
                            new_coordinate[1] = rand() % (MAP_SIZE-1);
                        }
                        hWallMatrix[new_coordinate[1]][new_coordinate[0]] = true;
=======
					{
						remove = rand() % 10;
						if (remove < remove_probab)
							hWallMatrix[tmp.x][tmp.y] = false;
>>>>>>> 3f52a0255d5685c5d67c0041cff32291fefce8b8
					}
				}
			}
			if (tmp.y - 1 > -1)
			{
				if (map[tmp.x][tmp.y - 1] != order[blkID].first)
				{
					if (vWallMatrix[tmp.x][tmp.y - 1] == true) // 좌
<<<<<<< HEAD
					{
                        new_coordinate[0] = rand()%(MAP_SIZE-1);
                        new_coordinate[1] = rand()%MAP_SIZE;
                        vWallMatrix[tmp.x][tmp.y - 1] = false;
                        while(vWallMatrix[new_coordinate[1]][new_coordinate[0]])
                        {
                            new_coordinate[0] = rand() % (MAP_SIZE-1);
                            new_coordinate[1] = rand() % MAP_SIZE;
                        }
                        vWallMatrix[new_coordinate[1]][new_coordinate[0]] = true;
=======
					{
						remove = rand() % 10;
						if (remove < remove_probab)
							vWallMatrix[tmp.x][tmp.y - 1] = false;
>>>>>>> 3f52a0255d5685c5d67c0041cff32291fefce8b8
					}
				}
			}
			if (col > tmp.y + 1)
			{
				if (map[tmp.x][tmp.y + 1] != order[blkID].first)
				{
					if (vWallMatrix[tmp.x][tmp.y] == true) // 우
<<<<<<< HEAD
					{
                        new_coordinate[0] = rand()%(MAP_SIZE-1);
                        new_coordinate[1] = rand()%MAP_SIZE;
                        vWallMatrix[tmp.x][tmp.y] = false;
                        while(vWallMatrix[new_coordinate[1]][new_coordinate[0]])
                        {
                            new_coordinate[0] = rand() % (MAP_SIZE-1);
                            new_coordinate[1] = rand() % MAP_SIZE;
                        }
                        vWallMatrix[new_coordinate[1]][new_coordinate[0]] = true;
=======
					{
						remove = rand() % 10;
						if (remove < remove_probab)
							vWallMatrix[tmp.x][tmp.y] = false;
>>>>>>> 3f52a0255d5685c5d67c0041cff32291fefce8b8
					}
				}
			}
		}
		tmp_vec.clear();
		int tmp = num_block;
		if (check_map() < tmp)
			exit_cond = true;
	}
	order.clear();
}
int ff::wall_num(char c)
{
	int cnt = 0;
	if (c == 'v')
	{
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col - 1; j++)
			{
				if (vWallMatrix[i][j] == 1)
					cnt++;
			}
		}
	}
	else if (c == 'h')
	{
		for (int i = 0; i < row - 1; i++)
		{
			for (int j = 0; j < col; j++)
			{
				if (hWallMatrix[i][j] == 1)
					cnt++;
			}
		}
	}
	return cnt;
}
