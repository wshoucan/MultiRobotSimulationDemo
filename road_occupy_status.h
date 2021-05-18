#ifndef _ROAD_OCCUPY_STATUS_H
#define _ROAD_OCCUPY_STATUS_H

#include "define.h"
#include "display.h"
#include "workshop.h"

class RoadOccupyStatus
{
private:
	int status[ROWS][COLS];   //通道栅格当前的状态 
	int ID[ROWS][COLS];    //占用当前通道的机器人的编号 
public:
	RoadOccupyStatus();   //初始化每个道路栅格的当前状态 ,初始化时全部为empty 
	void DrawRoadStatus();   //绘制道路状态图 
	void SetRoadOccupyStatus(int row, int col, int cell_status, int robot_ID);   //设置某个道路栅格的占用状态，以及占用该栅格的机器人的ID
	void SetRoadOccupyStatusToFree(int row, int col);  //将某个道路栅格的占用状态置为空闲
	int GetStatus(int row, int col);  
	int GetID(int row, int col);
};

extern RoadOccupyStatus road_occupy_status;

#endif

