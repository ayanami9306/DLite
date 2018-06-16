#include "FloodFill.h"
#include <iostream>
#include <time.h>
#include <vector>
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
	for (int ii = 0; ii < row; ii++)
	{
		for (int jj = 0; jj < col; jj++)
		{
			cout.width(3);
			cout << map[ii][jj] << " ";
			cout.width();
			if (jj < col - 1)
			{
				if (vWallMatrix[ii][jj] == true)
					cout << "|";
				else
					cout << " ";
			}
		}
		cout << endl;
		for (int jj = 0; jj < col; jj++)
		{
			if (ii < row - 1)
			{
				if (hWallMatrix[ii][jj] == true)
					cout << "----";
				else
					cout << "    ";
				if (jj < col - 1)
					cout << ".";
			}
		}
		cout << endl << endl;
	}
}

typedef struct coordi
{
	int x;
	int y;
}coordi;
void ff::edit_map(int level)
{
	for (int blkID = 2; blkID <= num_block; blkID++)
	{
		vector<coordi> tmp_vec;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				if (map[i][j] == blkID)
				{
					coordi tmp;
					tmp.x = i;
					tmp.y = j;
					tmp_vec.push_back(tmp); // ���е� block ��ġ ���� ����
				}
			}
		}
		// H:50%, N:66%, E:90% Ȯ���� �� ���� ��� ����
		for (int id = 0; id < tmp_vec.size(); id++)
		{
			int freq = rand() % 10;
			if (freq > level) 
				tmp_vec.erase(tmp_vec.begin() + id);
		}
		// �� ���� ����� ��濡 ���� ���� ������ �� 20%Ȯ���� �� ����
		int remove_probab = 2;
		for (int id = 0; id < tmp_vec.size(); id++)
		{
			int remove;
			coordi tmp = tmp_vec.data()[id];
			if (tmp.x - 1 > -1)
			{
				if (hWallMatrix[tmp.x - 1][tmp.y] == true) // ��
				{
					remove = rand() % 10;
					if (remove < remove_probab)
						hWallMatrix[tmp.x - 1][tmp.y] = false;
				}
			}
			if (row > tmp.x + 1)
			{
				if (hWallMatrix[tmp.x][tmp.y] == true) // ��
				{
					remove = rand() % 10;
					if (remove < remove_probab)
						hWallMatrix[tmp.x][tmp.y] = false;
				}
			}
			if (tmp.y - 1 > -1)
			{
				if (vWallMatrix[tmp.x][tmp.y - 1] == true) // ��
				{
					remove = rand() % 10;
					if (remove < remove_probab)
						vWallMatrix[tmp.x][tmp.y - 1] = false;
				}
			}
			if (col > tmp.y + 1)
			{
				if (vWallMatrix[tmp.x][tmp.y] == true) // ��
				{
					remove = rand() % 10;
					if (remove < remove_probab)
						vWallMatrix[tmp.x][tmp.y] = false;
				}
			}
		}
		tmp_vec.clear();
	}
}
