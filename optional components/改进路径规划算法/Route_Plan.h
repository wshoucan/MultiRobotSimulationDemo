#ifndef _ROUTE_PLAN_H
#define _ROUTE_PLAN_H

//#include "global.h"
#include "task.h"
#include "Jam_Control.h"
#include "workshop.h"

struct RouteRecord  //每条路径的记录
{
	int row[300];
	int col[300];
	int cell_number;
};

struct Crossroad
{
	int row;    //路口所在的行 
	int col;    //路口所在的列 
	float fn;     //从起点开始经过这一点到终点的总代价 
	float gn;     //起点到当前点的真实路径代价 
	float hn;     //当前点到终点的估算路径代价 
	int parent;     //当前点的父节点，是节点在open表中的编号 
	int inopen;     //该节点是否在open表中，1代表在open表，0代表在不在open表 
};

class RoutePlan
{
	Crossroad openlist[1000];
    Crossroad closelist[1000];

public:
	RouteRecord CreateRoute(Task task); // 根据路径规划的种类，选取对应的算法，生成路径

    RouteRecord AStarPathPlanning(Task task); //标准A*路径规划算法
    bool AStarPushIntoOpenlist(Task task, int mintag, int len, int clen, int p_row, int p_col); // 标准A*路径规划算法：将节点添加进OpenList
    bool AStarJudgeCellType(Task task, int mintag, int len, int clen, int direction); // 标准A*路径规划算法：判断路口的种类（即路口的上下左右方向是否允许通行）

    RouteRecord ImprovedAStarPathPlanning(Task task);  // 改进A*路径规划算法
    RouteRecord AddIntoRouteRecord(RouteRecord, int);  // 改进A*路径规划算法：将非路口节点添加进路径序列
    RouteRecord CreateCrossRoute(int, int, int, int);  // 改进A*路径规划算法：生成从起点路口到终点路口的路口序列 
    int JudgeTurnValue(int last_row, int last_col, int pre_row, int pre_col, int next_row, int next_col);  // 改进A*路径规划算法：判断三点是否共线，返回转弯代价
	int GetNextCrossLeftRight(int, int);  // 改进A*路径规划算法：确定左右方向的方向
	int GetNextCrossUpDown(int,int);  // 改进A*路径规划算法：确定上下方向的方向

	RouteRecord AddTurnActionToRouteRecord(RouteRecord primary_route);  // 将实际的转弯动作添加进路径记录中，各种路径规划算法生成路径后都要调用本函数


};

#endif
