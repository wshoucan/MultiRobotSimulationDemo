#include "Route_Plan.h"

using namespace std;

// ����·���滮�����࣬ѡȡ��Ӧ���㷨������·��
RouteRecord RoutePlan::CreateRoute(Task task)
{
	switch (PATH_PLANNING_ALGORITHM_TYPE)
	{
	case A_STAR:
		return AStarPathPlanning(task);
		break;

	}
}

// ��׼A*·���滮�㷨
RouteRecord RoutePlan::AStarPathPlanning(Task task)
{
	openlist[0].row = task.get_begin_row();   //������ʼ��������open�� 
	openlist[0].col = task.get_begin_col();
	openlist[0].gn = 0;
	openlist[0].hn = abs(task.get_end_row() - task.get_begin_row()) + abs(task.get_end_col() - task.get_begin_col());
	openlist[0].fn = openlist[0].gn + openlist[0].hn;
	openlist[0].inopen = 1;
	openlist[0].parent = 0;

	int endflag;
	int len = 1;  //openlistlength     openlist���м���Ԫ�أ�len�͵��ڼ� 
	int clen = 0;   //closelistlength      closelist���м���Ԫ�أ�clen�͵��ڼ� 

	while (1)
	{
		// �ҵ�openlist��inopen==1����fn��С��Ԫ��
		int minfn;
		int mintag = 0;
		while (1)    //��openlist�����ҵ���һ��inopen==1��Ԫ�� 
		{
			if (openlist[mintag].inopen == 0)
				mintag++;
			else
			{
				minfn = openlist[mintag].fn;
				break;
			}
		}
		for (int i = mintag; i < len; i++)   //�ҵ��ܴ���fn��С��Ԫ�أ�Ԫ����openlist���е�λ��Ϊmintag 
		{
			if (openlist[i].inopen == 1 && openlist[i].fn < minfn)
			{
				mintag = i;
				minfn = openlist[i].fn;
			}
		}

		if (openlist[mintag].row == task.get_end_row() && openlist[mintag].col == task.get_end_col())  //���fn��С�ĵ����յ㣬Ѱ·���� 
		{
			endflag = mintag;
			break;
		}

		if (AStarJudgeCellType(task, mintag, len, clen, UP_ROAD) == true)
			len++;
		if (AStarJudgeCellType(task, mintag, len, clen, DOWN_ROAD) == true)
			len++;
		if (AStarJudgeCellType(task, mintag, len, clen, LEFT_ROAD) == true)
			len++;
		if (AStarJudgeCellType(task, mintag, len, clen, RIGHT_ROAD) == true)
			len++;

		openlist[mintag].inopen = 0;   //��fn��С�ĵ��Ƶ�closelist ��ʵ������㻹��openlist�У�ֻ��inopen����Ϊ0 
		closelist[clen] = openlist[mintag];
		clen++;
	}

	// ���յ㿪ʼ��Ҳ����openlist[endflag],���β��Ҹ��ڵ㣬�����¼������·�ڵ�·������¼��record�� 
	RouteRecord record;
	int temp = endflag;
	record.cell_number = 0;
	while (1)
	{
		if (openlist[temp].row == task.get_begin_row() && openlist[temp].col == task.get_begin_col())
		{
			record.row[record.cell_number] = openlist[temp].row;
			record.col[record.cell_number] = openlist[temp].col;
			record.cell_number++;
			break;
		}
		else
		{
			record.row[record.cell_number] = openlist[temp].row;
			record.col[record.cell_number] = openlist[temp].col;
			record.cell_number++;
			temp = openlist[temp].parent;
		}
	}

	//˳���¼������·�ڣ�Ҳ���ǰ�record�����������reall_record 
	RouteRecord reall_record;
	reall_record.cell_number = 0;
	for (int i = record.cell_number - 1; i >= 0; i--)
	{
		reall_record.row[reall_record.cell_number] = record.row[i];
		reall_record.col[reall_record.cell_number] = record.col[i];
		reall_record.cell_number++;
	}

	return AddTurnActionToRouteRecord(reall_record);   //��·����ת��·�ڣ�����ʵ�ʵ�ת�䶯��
}

// ��׼A*·���滮�㷨�����ڵ���ӽ�OpenList
bool RoutePlan::AStarPushIntoOpenlist(Task task, int mintag, int len, int clen, int p_row, int p_col)  //�˴���p_row��p_col�ǵȴ�����openlist�е� 
{
	int closeflag = 0;  //��ǰ�ڵ��ڲ���closelist�е�������0�����ڣ�1������ 
	int openflag = 0;
	int num;    //��ǰ�ڵ���open���У���¼�����ڵ�λ�� 

	for (int i = 0; i < len; i++)   //�жϵ�ǰ�ڵ��ڲ���closelist�� 
	{
		if (openlist[i].row == p_row && openlist[i].col == p_col && openlist[i].inopen == 0)
		{
			closeflag = 1;
			return false;
		}
	}

	for (int i = 0; i < len; i++)    //�жϵ�ǰ�ڵ��ڲ���openlist�� 
	{
		if (openlist[i].row == p_row && openlist[i].col == p_col && openlist[i].inopen == 1)
		{
			openflag = 1;
			num = i;
			break;
		}
	}

	if (openflag == 0)
	{
		openlist[len].row = p_row;   //�������open���У��Ͱѵ�ǰ�ڵ����openlist 
		openlist[len].col = p_col;
		openlist[len].gn = openlist[mintag].gn + 1;
		openlist[len].hn = abs(task.get_end_row() - p_row) + abs(task.get_end_col() - p_col);
		openlist[len].fn = openlist[len].gn + openlist[len].hn;
		openlist[len].inopen = 1;
		openlist[len].parent = mintag;
		return true;    //open�����������Ԫ�ء� 
	}
	else
	{                 //�����openlist�У��ͱȽ�gn����ǰgn��С�͸��ĸ�ĸ 
		if (openlist[mintag].gn + 1 < openlist[num].gn)
		{
			openlist[num].parent = mintag;
			return false;   //open����û�������Ԫ�� 
		}
	}
}

// ��׼A*·���滮�㷨���ж�·�ڵ����ࣨ��·�ڵ��������ҷ����Ƿ�����ͨ�У�
bool RoutePlan::AStarJudgeCellType(Task task, int mintag, int len, int clen, int direction)
{
	int p_row = openlist[mintag].row;
	int p_col = openlist[mintag].col;
	switch (direction)
	{
	case UP_ROAD:
		if (cell[1][p_col].kind == UP_ROAD && p_row > 0)
			if (AStarPushIntoOpenlist(task, mintag, len, clen, openlist[mintag].row - 1, openlist[mintag].col) == true)
				return true;
			else
				return false;
		break;

	case DOWN_ROAD:
		if (cell[1][p_col].kind == DOWN_ROAD && p_row < ROWS - 1)
			if (AStarPushIntoOpenlist(task, mintag, len, clen, openlist[mintag].row + 1, openlist[mintag].col) == true)
				return true;
			else
				return false;
		break;

	case LEFT_ROAD:
		if (cell[p_row][1].kind == LEFT_ROAD && p_col > 0)
			if (AStarPushIntoOpenlist(task, mintag, len, clen, openlist[mintag].row, openlist[mintag].col - 1) == true)
				return true;
			else
				return false;
		break;

	case RIGHT_ROAD:
		if (cell[p_row][1].kind == RIGHT_ROAD && p_col < COLS - 1)
			if (AStarPushIntoOpenlist(task, mintag, len, clen, openlist[mintag].row, openlist[mintag].col + 1) == true)
				return true;
			else
				return false;
		break;
	}

}

// ��ʵ�ʵ�ת�䶯����ӽ�·����¼�У�����·���滮�㷨����·����Ҫ���ñ�����
RouteRecord RoutePlan::AddTurnActionToRouteRecord(RouteRecord primary_route)
{
	RouteRecord new_route;
	new_route.cell_number = 1;
	new_route.row[0] = primary_route.row[0];
	new_route.col[0] = primary_route.col[0];
	for (int i = 1; i < primary_route.cell_number - 1; i++)
	{
		if (abs(primary_route.row[i - 1] - primary_route.row[i + 1]) == 1 && abs(primary_route.col[i - 1] - primary_route.col[i + 1]) == 1)
		{
			for (int j = 0;j < TURN_COST + 1; j++)
			{
				new_route.row[new_route.cell_number] = primary_route.row[i];
				new_route.col[new_route.cell_number] = primary_route.col[i];
				new_route.cell_number++;
			}
		}
		else
		{
			new_route.row[new_route.cell_number] = primary_route.row[i];
			new_route.col[new_route.cell_number] = primary_route.col[i];
			new_route.cell_number++;
		}
	}
	new_route.row[new_route.cell_number] = primary_route.row[primary_route.cell_number - 1];
	new_route.col[new_route.cell_number] = primary_route.col[primary_route.cell_number - 1];
	new_route.cell_number++;

	return new_route;
}