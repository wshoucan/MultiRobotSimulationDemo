#ifndef _INIT_WORKSHOP_H
#define _INIT_WORKSHOP_H

#include <iostream>  
#include "define.h"
#include "display.h"

struct WorkShop
{
public:
	int number;  //����դ��ı�� 0-1000
	int kind;  //դ�������  1001-1010
};

void InitMap();   //��ʼ����ͼ 
void DrawMap();   //����դ���ͼ 

extern WorkShop cell[ROWS][COLS];    //��ͼ��ÿ��դ������� 

#endif











