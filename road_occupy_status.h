#ifndef _ROAD_OCCUPY_STATUS_H
#define _ROAD_OCCUPY_STATUS_H

#include "define.h"
#include "display.h"
#include "workshop.h"

class RoadOccupyStatus
{
private:
	int status[ROWS][COLS];   //ͨ��դ��ǰ��״̬ 
	int ID[ROWS][COLS];    //ռ�õ�ǰͨ���Ļ����˵ı�� 
public:
	RoadOccupyStatus();   //��ʼ��ÿ����·դ��ĵ�ǰ״̬ ,��ʼ��ʱȫ��Ϊempty 
	void DrawRoadStatus();   //���Ƶ�·״̬ͼ 
	void SetRoadOccupyStatus(int row, int col, int cell_status, int robot_ID);   //����ĳ����·դ���ռ��״̬���Լ�ռ�ø�դ��Ļ����˵�ID
	void SetRoadOccupyStatusToFree(int row, int col);  //��ĳ����·դ���ռ��״̬��Ϊ����
	int GetStatus(int row, int col);  
	int GetID(int row, int col);
};

extern RoadOccupyStatus road_occupy_status;

#endif

