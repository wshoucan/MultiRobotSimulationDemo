#include "road_occupy_status.h"

using namespace std;

RoadOccupyStatus road_occupy_status;

// ��ʼ��ȫ����·դ���״̬Ϊ����
RoadOccupyStatus::RoadOccupyStatus()
{
	for (int i = 0;i < ROWS;i++)
	{
		for (int j = 0;j < COLS;j++)
		{
			if (cell[i][j].kind != WORKBENCH)     //���ǻ���դ��͸����ֵ 
				status[i][j] = ROAD_FREE;
		}
	}
}

// ���õ�·դ��ĵ�ǰ״̬
void RoadOccupyStatus::SetRoadOccupyStatus(int row, int col, int cell_status, int robot_ID)
{
	status[row][col] = cell_status;
	ID[row][col] = robot_ID;
}

// ��ĳ����·դ���״̬��Ϊ����
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

//������·״̬ͼ����ҪΪ�˵��Գ����� 
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