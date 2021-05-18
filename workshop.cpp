#include "workshop.h"

using namespace std;

WorkShop cell[ROWS][COLS];    //��ͼ��ÿ��դ������� 

// ��ʼ��������ͼ
void InitMap()
{
	int kind_temp[ROWS][COLS];
	for (int i = 0; i < ROWS; i++)
		for (int j = 0; j < COLS; j++)
		{
			if (i % (WORKBENCH_LENGTH + 1) == 0 || j % (WORKBENCH_WIDTH + 1) == 0)    //��ͨ�� 
			{
				if (i % (WORKBENCH_LENGTH + 1) == 0 && j % (WORKBENCH_WIDTH + 1) == 0)   //��·��
					kind_temp[i][j] = CROSS;
				else
				{
					if (i % (WORKBENCH_LENGTH + 1) == 0)   //�ǳ�·����ĺ���ͨ�� 
					{
						if (i % ((WORKBENCH_LENGTH + 1) * 2) == 0)
							kind_temp[i][j] = RIGHT_ROAD;
						else
							kind_temp[i][j] = LEFT_ROAD;
					}
					else    //�ǳ�·���������ͨ�� 
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

// ���Ƴ�ʼ���ĵ�ͼ 
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
				cout << "  ";	  //��
				break;
			case DOWN_ROAD:
				cout << "  ";	  //��
				break;
			case LEFT_ROAD:
				cout << "  ";	  //��
				break;
			case RIGHT_ROAD:
				cout << "  ";	  //��
				break;

			case WORKBENCH:
				cout << "��";
				break;

			}
		}
		cout << endl;
	}
}

