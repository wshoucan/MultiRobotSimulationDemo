#ifndef _CREATE_TASK_H
#define _CREATE_TASK_H

#include <stdlib.h>   //生成随机数 
#include "define.h"

class Task   //任务的起点与终点 
{
private:
	int begin_row;
	int begin_col;
	int end_row;
	int end_col;
	int priority;
public:
	Task CreateTask();  //生成一个随机任务（没有充电区域）
	void SetTask(int _begin_row, int _begin_col, int _end_row, int _end_col, int _priority);
	int get_begin_row() { return begin_row; }
	int get_begin_col() { return begin_col; }
	int get_end_row() { return end_row; }
	int get_end_col() { return end_col; }
	int get_priority() { return priority; }
};

#endif