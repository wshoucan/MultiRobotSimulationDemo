#include "workshop.h"

using namespace std;

WorkShop cell[ROWS][COLS];    //地图中每个栅格的种类 

// 初始化工作地图
void InitMap()
{
	int kind_temp[ROWS][COLS];
	for (int i = 0; i < ROWS; i++)
		for (int j = 0; j < COLS; j++)
		{
			if (i % (WORKBENCH_LENGTH + 1) == 0 || j % (WORKBENCH_WIDTH + 1) == 0)    //是通道 
			{
				if (i % (WORKBENCH_LENGTH + 1) == 0 && j % (WORKBENCH_WIDTH + 1) == 0)   //是路口
					kind_temp[i][j] = CROSS;
				else
				{
					if (i % (WORKBENCH_LENGTH + 1) == 0)   //是除路口外的横向通道 
					{
						if (i % ((WORKBENCH_LENGTH + 1) * 2) == 0)
							kind_temp[i][j] = RIGHT_ROAD;
						else
							kind_temp[i][j] = LEFT_ROAD;
					}
					else    //是除路口外的纵向通道 
					{
						if (j % ((WORKBENCH_WIDTH + 1) * 2) == 0)
							kind_temp[i][j] = UP_ROAD;
						else
							kind_temp[i][j] = DOWN_ROAD;
					}
				}
			}
			else
			{
				kind_temp[i][j] = WORKBENCH;
			}
		}

	for (int i = 0;i < ROWS;i++)
	{
		for (int j = 0;j < COLS;j++)
		{
			cell[i][j].number = i * COLS + j;
			cell[i][j].kind = kind_temp[i][j];
		}
	}
}

// 绘制初始化的地图 
void DrawMap()
{
	int i, j;
	for (i = 0;i < ROWS;i++)
	{
		for (j = 0;j < COLS;j++)
		{
			switch (cell[i][j].kind)
			{
			case CROSS:
				cout << "  ";
				break;
			case UP_ROAD:
				cout << "  ";	  //↑
				break;
			case DOWN_ROAD:
				cout << "  ";	  //↓
				break;
			case LEFT_ROAD:
				cout << "  ";	  //←
				break;
			case RIGHT_ROAD:
				cout << "  ";	  //→
				break;

			case WORKBENCH:
				cout << "■";
				break;

			}
		}
		cout << endl;
	}
}

