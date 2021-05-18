#include "road_occupy_status.h"

using namespace std;

RoadOccupyStatus road_occupy_status;

// 初始化全部道路栅格的状态为空闲
RoadOccupyStatus::RoadOccupyStatus()
{
	for (int i = 0;i < ROWS;i++)
	{
		for (int j = 0;j < COLS;j++)
		{
			if (cell[i][j].kind != WORKBENCH)     //不是货物栅格就赋予空值 
				status[i][j] = ROAD_FREE;
		}
	}
}

// 设置道路栅格的当前状态
void RoadOccupyStatus::SetRoadOccupyStatus(int row, int col, int cell_status, int robot_ID)
{
	status[row][col] = cell_status;
	ID[row][col] = robot_ID;
}

// 将某个道路栅格的状态置为空闲
void RoadOccupyStatus::SetRoadOccupyStatusToFree(int row, int col)
{
	status[row][col] = ROAD_FREE;
	ID[row][col] = ROBOT_NUM + 1;
}

int RoadOccupyStatus::GetStatus(int row, int col)
{
	return status[row][col];
}

int RoadOccupyStatus::GetID(int row, int col)
{
	return ID[row][col];
}

//画出道路状态图，主要为了调试程序用 
void RoadOccupyStatus::DrawRoadStatus()
{
	display.HiddenCursor();
	for (int i = 0; i < ROWS; i++)
	{
		display.GotoXY(i, 70);
		for (int j = 0; j < COLS; j++)
		{
			if (cell[i][j].kind != WORKBENCH)
				cout << status[i][j] << " ";
			else
				cout << "  ";
		}
		cout << endl;
	}
}