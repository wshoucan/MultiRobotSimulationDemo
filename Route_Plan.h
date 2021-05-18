#ifndef _ROUTE_PLAN_H
#define _ROUTE_PLAN_H

#include "task.h"
#include "workshop.h"

struct RouteRecord  //ÿ��·���ļ�¼
{
	int row[300];
	int col[300];
	int cell_number;
};

struct Crossroad
{
	int row;    //·�����ڵ��� 
	int col;    //·�����ڵ��� 
	float fn;     //����㿪ʼ������һ�㵽�յ���ܴ��� 
	float gn;     //��㵽��ǰ�����ʵ·������ 
	float hn;     //��ǰ�㵽�յ�Ĺ���·������ 
	int parent;     //��ǰ��ĸ��ڵ㣬�ǽڵ���open���еı�� 
	int inopen;     //�ýڵ��Ƿ���open���У�1������open��0�����ڲ���open�� 
};

class RoutePlan
{
	Crossroad openlist[1000];
    Crossroad closelist[1000];

public:
	RouteRecord CreateRoute(Task task); // ����·���滮�����࣬ѡȡ��Ӧ���㷨������·��

    RouteRecord AStarPathPlanning(Task task); //��׼A*·���滮�㷨
    bool AStarPushIntoOpenlist(Task task, int mintag, int len, int clen, int p_row, int p_col); // ��׼A*·���滮�㷨�����ڵ���ӽ�OpenList
    bool AStarJudgeCellType(Task task, int mintag, int len, int clen, int direction); // ��׼A*·���滮�㷨���ж�·�ڵ����ࣨ��·�ڵ��������ҷ����Ƿ�����ͨ�У�

	RouteRecord AddTurnActionToRouteRecord(RouteRecord primary_route);  // ��ʵ�ʵ�ת�䶯����ӽ�·����¼�У�����·���滮�㷨����·����Ҫ���ñ�����


};

#endif
