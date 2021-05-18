#ifndef _INIT_WORKSHOP_H
#define _INIT_WORKSHOP_H

#include <iostream>  
#include "define.h"
#include "display.h"

struct WorkShop
{
public:
	int number;  //各个栅格的编号 0-1000
	int kind;  //栅格的种类  1001-1010
};

void InitMap();   //初始化地图 
void DrawMap();   //绘制栅格地图 

extern WorkShop cell[ROWS][COLS];    //地图中每个栅格的种类 

#endif











