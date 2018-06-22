#pragma once
#include <stdio.h>
#include "RobotClass.h"
#include "Variables.h"

void task_alloc();
bool check_reservation(int robotID, int taskID);
void alloc_new_task(int index);
