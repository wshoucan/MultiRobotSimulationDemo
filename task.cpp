#include "task.h"

using namespace std;

// ����һ���������û�г������
Task Task::CreateTask()   
{
	Task task;
	int task_row_num = WORKBENCH_LENGTH * WORKBENCH_LENGTH_NUMBER;
	int task_col_num = WORKBENCH_WIDTH_NUMBER + 1;
	int task_row[WORKBENCH_LENGTH * WORKBENCH_LENGTH_NUMBER];  //���������кż��� 
	int task_col[WORKBENCH_WIDTH_NUMBER + 1];   //���������кż��� 

	for (int i = 0, j = 0; i < ROWS; i++)   //i����ѭ��������j����������Ԫ�ظ��� 
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
	task.priority = rand() % 4 + 1;   //�������ȼ� 
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