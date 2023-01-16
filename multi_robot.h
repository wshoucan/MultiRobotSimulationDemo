#ifndef _MULTI_ROBOT_H
#define _MULTI_ROBOT_H

#include <iostream>
#include <windows.h>
#include <stdlib.h>   //生成随机数 
#include <time.h>    
#include <math.h> 

#include "route_Plan.h"
#include "display.h"
#include "workshop.h"

class Robot
{
private:
	int ID;  //机器人的编号
	int p_row;    //机器人当前所处位置的行 
	int p_col;    //机器人当前所处位置的列 
	int status;   //机器人的当前状态 
	Task real_task;    //实际的搬运任务，每次获取新的搬运任务时才赋值
	Task task;     //机器人当前在执行的任务 ,可能是调度，也可能是搬运，与route_record对应
	RoutePlan route_plan;  //专门用来做路径规划的生成对象
	RouteRecord route_record;   //机器人当前的行进路线 
	int p_num;     //机器人当前执行到了本任务的第几步 
	int next_step_flag;  //机器人下一步应该怎么移动的指示

public:
	int get_next_step()
	{
		return next_step_flag;
	}
	int get_p_row()
	{
		return p_row;
	}
	int get_p_col()
	{
		return p_col;
	}

    Robot();  //构造函数
	void InitRobot(int _ID); //对机器人初始化 
	int FirstTaskToRobot(Task task,int task_num);  //系统初始化后，给每个机器人赋予一个初始任务
    void TaskToRobot(Task);   //将一个任务赋予一个机器人 
    void JudgeNextStep();   //判断下一步的种类
    void MoveSingleStep();    //将编号为ID的机器人进行一次操作，如何操作根据next_step决定 
    int RobotStatusTransform(Task task);    //JudgeNextStep()函数返回值为1时，进行状态转换
};

extern Robot* robot;   //生成多个仿真机器人 

#endif
