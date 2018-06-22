#pragma once
#ifndef FLOOD_FILL
#define FLOOD_FILL
#include "Variables.h"

class ff
{
public:
	ff(int, int);
	~ff();
	int check_map();
	void print_map();
	void edit_map(int);
	int wall_num(char);
	
private:
	void clear_map();
	void flood_loop(int, int, int color);

private:
	int** map;
	int row, col;
	int num_block;
};

#endif
