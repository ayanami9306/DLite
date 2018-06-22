#pragma once
#include <stdio.h>
#include "RobotClass.h"
#include "Variables.h"

typedef struct edge
{
    int index[2];
    double dis;
}edge;

typedef struct task_to_task_cost
{
    int from, to;
    int distance;
}task_to_task_cost;

bool cmp_task_task(task_to_task_cost a, task_to_task_cost b);

int find(int u, int *parent);

bool merge(int u, int v, int *parent);

void draw_MSTree();
