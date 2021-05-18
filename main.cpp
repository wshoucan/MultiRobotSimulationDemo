#include "main.h"

using namespace std;

int main()
{
	display.SetInterface();

	// 初始化地图，其余道路状态、充电点状态、机器人数据输出文件的初始化都在构造函数中自动进行了
	InitMap();
	DrawMap();

   // 生成一批随机任务
	srand(3);   //srand((unsigned)time(NULL));
	Task *task = new Task[MAX_TASK_NUM];
	for (int i = 0; i < MAX_TASK_NUM; i++)     
		task[i] = task[i].CreateTask();

   // 对所有机器人进行初始化，并在屏幕上输出
	for (int ID = 0; ID < ROBOT_NUM; ID++)    
	{
		robot[ID].InitRobot(ID);
	}

    int clock = 0;   //时钟周期
	int task_num = 0;  //现在执行到第几个task了 
	int complete_task_num = 0;  //已经完成的任务的个数

    // 遍历一遍机器人，把第一个任务分配给每一个机器人 
    for (int ID = 0; ID < ROBOT_NUM; ID++)    
    {
	    task_num = robot[ID].FirstTaskToRobot(task[task_num], task_num);
    }

    // 对每个机器人进行遍历，满足条件就向前行进一步 
	while (1)    
	{

		clock++;
		//DrawRoadStatus();

		// 判断下一步应该怎么走
		for (int ID = 0; ID < ROBOT_NUM; ID++)
		{
			robot[ID].JudgeNextStep();
		}

		// 实际移动一步，如果当前位置是路径终点，就进入机器人状态转换
		for (int ID = 0; ID < ROBOT_NUM; ID++)
		{
            robot[ID].MoveSingleStep();
			if (robot[ID].get_next_step() == 1)
			{
				if (robot[ID].RobotStatusTransform(task[task_num]))
				{
					complete_task_num++;
             		task_num++;
				}
			}
		}
		Sleep(50);
	}
}