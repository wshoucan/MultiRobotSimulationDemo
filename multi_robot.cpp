#include "multi_robot.h"

using namespace std;

Robot *robot = new Robot[ROBOT_NUM];   //多个机器人 

// 构造函数
Robot::Robot()   
{
	status = ROBOT_FREE;
}

// 初始化机器人的编号与位置，并在屏幕上输出
void Robot::InitRobot(int _ID)
{
	ID = _ID;
	p_row = (ID % ((ROWS + 1) / 2)) * 2;
	p_col = (ID / ((ROWS + 1) / 2)) * 6;
	display.GotoXY(get_p_row(), get_p_col() * 2);
	display.SetColor(GREEN);
	cout << "●";
}

// 将第一个初始任务赋给机器人
int Robot::FirstTaskToRobot(Task task, int task_num)
{
	if (status == ROBOT_FREE)
	{
		real_task = task;
		Task remove_task;   //将机器人从当前位置调度到实际任务的起点
		remove_task.SetTask(p_row, p_col, real_task.get_begin_row(), real_task.get_begin_col(), real_task.get_priority());
		TaskToRobot(remove_task);
		status = ROBOT_SCHEDULED;
		return ++task_num;
	}
	else
		return task_num;
}

// 将任务赋予机器人 
void Robot::TaskToRobot(Task _task)
{
	task = _task;
	route_record = route_plan.CreateRoute(task);
	p_row = task.get_begin_row();
	p_col = task.get_begin_col();
	p_num = 0;       //p_num是从0开始的 
	road_occupy_status.SetRoadOccupyStatus(p_row, p_col,ROAD_RESERVED,ID);
}

// 判断下一步的种类
// 1：当前位置是本路径的最后一步，进入状态转换
// 2：下一步仍在路口进行转弯，保持原地不动
// 3：想前进一步，但下一栅格忙，原地等待
// 4：按路径前进一步
void Robot::JudgeNextStep()
{
	if (p_num == route_record.cell_number - 1)   //当前所处的位置已经是本路径的最后一步 {
	{
        next_step_flag = 1;  
		return;
	}
	else
	{
		int next_row = route_record.row[p_num + 1];
		int next_col = route_record.col[p_num + 1];
		if (p_row == next_row && p_col == next_col)  // 下一步仍在路口进行转弯，保持原地不动 
		{
            next_step_flag = 2;
			return;
		}   
		else
		{
			if (road_occupy_status.GetStatus(next_row, next_col) == ROAD_FREE)
			{
				road_occupy_status.SetRoadOccupyStatus(next_row, next_col,ROAD_RESERVED, ID);
				next_step_flag = 4;    //下一步想前进一步，并且下一栅格空闲，就前进一步	
				return;
			}
			else     //下一步想前进一步，但下一栅格忙 
			{
                next_step_flag = 3;
			    return;		
			}
		}
	}
}

 // 将编号为ID的机器人进行一次操作，如何操作根据next_step决定 
void Robot::MoveSingleStep()
{
	switch (next_step_flag)
	{

	case 1:  //当前所处的位置已经是本路径的最后一步 
		break;

	case 2:     //下一步仍在路口进行转弯，保持原地不动 
		display.GotoXY(robot[ID].p_row, robot[ID].p_col * 2);
		display.SetColor(GREEN);
		cout << "●";
		p_num++;
		break;

	case 3:    //下一步想前进一步，但栅格被占用，所以原地不动 
		display.GotoXY(p_row, p_col * 2);
		display.SetColor(YELLOW);
		cout << "●";
		break;

	case 4:    //下一步想前进一步，并且栅格空闲，所以前进一步
		display.GotoXY(p_row, p_col * 2);
		cout << "  ";
		road_occupy_status.SetRoadOccupyStatusToFree(p_row, p_col); //当前栅格的状态置为空闲 
		display.GotoXY(route_record.row[p_num + 1], route_record.col[p_num + 1] * 2);
		display.SetColor(GREEN);
		cout << "●";
		p_row = route_record.row[p_num + 1];
		p_col = route_record.col[p_num + 1];
		road_occupy_status.SetRoadOccupyStatus(p_row, p_col, ROAD_OCCUPIED, ID);  //下一栅格的状态置为占用 
		p_num++;
		break;
	}
}

 // JudgeNextStep(int ID)函数返回值为1时，进行状态转换
int Robot::RobotStatusTransform(Task p_task)
{
	int task_allocation_flag = 0;  //是否把传进来的任务给分配下去，分配成功则将flag置为1，只有case ROBOT_FREE 可能分配成功

	switch (status)
	{
	case ROBOT_CARRYING:  //当前是忙的最后一步，说明搬运任务执行完了，变成卸货
		status = ROBOT_FREE;
		task_allocation_flag = 1;
		real_task = p_task;
		Task remove_task;   //将机器人从当前位置调度到实际任务的起点
		remove_task.SetTask(p_row, p_col, real_task.get_begin_row(), real_task.get_begin_col(), real_task.get_priority());
		TaskToRobot(remove_task);
		status = ROBOT_SCHEDULED;
		break;

	case ROBOT_SCHEDULED:  //当前是调度的最后一步，说明调度完了，变成装货
		status = ROBOT_CARRYING;
		TaskToRobot(real_task);
		break;
	}
	return task_allocation_flag;
}
