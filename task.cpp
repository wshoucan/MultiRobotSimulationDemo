#include "task.h"

using namespace std;

// 生成一个随机任务（没有充电区域）
Task Task::CreateTask()   
{
	Task task;
	int task_row_num = WORKBENCH_LENGTH * WORKBENCH_LENGTH_NUMBER;
	int task_col_num = WORKBENCH_WIDTH_NUMBER + 1;
	int task_row[WORKBENCH_LENGTH * WORKBENCH_LENGTH_NUMBER];  //任务起点的行号集合 
	int task_col[WORKBENCH_WIDTH_NUMBER + 1];   //任务起点的列号集合 

	for (int i = 0, j = 0; i < ROWS; i++)   //i控制循环次数，j控制数组中元素个数 
	{
		if (i % (WORKBENCH_LENGTH + 1) != 0)
		{
			task_row[j] = i;
			j++;
		}
	}
	for (int i = 0, j = 0; i < COLS; i++)
	{
		if (i % (WORKBENCH_WIDTH + 1) == 0)
		{
			task_col[j] = i;
			j++;
		}
	}
	task.priority = rand() % 4 + 1;   //代表优先级 
	task.begin_row = task_row[rand() % task_row_num];
	task.begin_col = task_col[rand() % task_col_num];
	task.end_row = task_row[rand() % task_row_num];
	task.end_col = task_col[rand() % task_col_num];
	return task;
}

void Task::SetTask(int _begin_row, int _begin_col, int _end_row, int _end_col, int _priority)
{
	begin_row = _begin_row;
	begin_col = _begin_col;
	end_row = _end_row;
	end_col = _end_col;
	priority = _priority;
}