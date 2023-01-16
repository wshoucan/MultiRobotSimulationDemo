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

	case IMPROVER_A_STAR:
		return ImprovedAStarPathPlanning(task);
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



// �Ľ�A*·���滮�㷨
RouteRecord RoutePlan::ImprovedAStarPathPlanning(Task task)
{
	//cout << task.get_begin_row() << "  " << task.get_begin_col() << "  " << task.get_end_row() << "  " << task.get_end_col() << "  " << endl;
	RouteRecord route_record;    //��¼�����˾�����·�� 
	route_record.row[0] = task.get_begin_row();
	route_record.col[0] = task.get_begin_col();
	route_record.cell_number = 1;  //���������ߵĲ��� 

	int must_cross_row;   //��Ҫ�����յ�����뾭����·�ڵ��� 
	int must_cross_col = task.get_end_col();  //��Ҫ�����յ�����뾭����·�ڵ��� 
	if (cell[task.get_end_row()][task.get_end_col()].kind == UP_ROAD)    //��Ϊ����·��������·�� 
		must_cross_row = ((int)(task.get_end_row() / (WORKBENCH_LENGTH + 1) + 1)) * (WORKBENCH_LENGTH + 1);
	else
		must_cross_row = ((int)(task.get_end_row() / (WORKBENCH_LENGTH + 1))) * (WORKBENCH_LENGTH + 1);

	int direction;  //����ָʾ��һ�����ߵķ���
	int p_row = route_record.row[0];       //����������λ�� 
	int p_col = route_record.col[0];

    //������յ���ͬһ�������ϣ����ҷ���ɴ�
	if ((int)(task.get_begin_row() / (WORKBENCH_LENGTH + 1)) == (int)(task.get_end_row() / (WORKBENCH_LENGTH + 1)) && task.get_begin_col() == task.get_end_col() //�������ֻ��˵����ͬһС����������� 
		&& ((task.get_end_row() - task.get_begin_row() <= 0 && cell[p_row][p_col].kind == UP_ROAD)
			|| (task.get_end_row() - task.get_begin_row() >= 0 && cell[p_row][p_col].kind == DOWN_ROAD)))
	{
		if (task.get_end_row() - task.get_begin_row() <= 0 && cell[p_row][p_col].kind == UP_ROAD)  //������յ����� 
			direction = UP;
		else
			direction = DOWN;

		while ((p_row != task.get_end_row()) || (p_col != task.get_end_col()))  //�ж��Ƿ񵽴��յ� 
		{
			route_record = AddIntoRouteRecord(route_record, direction);
			p_row = route_record.row[route_record.cell_number - 1];
			p_col = route_record.col[route_record.cell_number - 1];
		}
		return route_record;
	}

	//������յ���ͬһ�������ϣ����ҷ���ɴ�
	if ((int)(task.get_begin_col() / (WORKBENCH_WIDTH + 1)) == (int)(task.get_end_col() / (WORKBENCH_WIDTH + 1)) && task.get_begin_row() == task.get_end_row() //�������ֻ��˵����ͬһС����������� 
		&& ((task.get_end_col() - task.get_begin_col() <= 0 && cell[p_row][p_col].kind == LEFT_ROAD)
			|| (task.get_end_col() - task.get_begin_col() >= 0 && cell[p_row][p_col].kind == RIGHT_ROAD)))
	{
		if (task.get_end_col() - task.get_begin_col() <= 0 && cell[p_row][p_col].kind == LEFT_ROAD)  //������յ����� 
			direction = LEFT;
		else
			direction = RIGHT;

		while ((p_row != task.get_end_row()) || (p_col != task.get_end_col()))  //�ж��Ƿ񵽴��յ� 
		{
			route_record = AddIntoRouteRecord(route_record, direction);
			p_row = route_record.row[route_record.cell_number - 1];
			p_col = route_record.col[route_record.cell_number - 1];
		}
		return route_record;
	}

    // ������յ㲻��ͬһС����

    //�����һ��·�� 
	while (cell[p_row][p_col].kind != CROSS)   
	{
		if (cell[p_row][p_col].kind == UP_ROAD)
			direction = UP;
		if (cell[p_row][p_col].kind == DOWN_ROAD)
			direction = DOWN;
		if (cell[p_row][p_col].kind == LEFT_ROAD)
			direction = LEFT;
		if (cell[p_row][p_col].kind == RIGHT_ROAD)
			direction = RIGHT;

		route_record = AddIntoRouteRecord(route_record, direction);
		p_row = route_record.row[route_record.cell_number - 1];
		p_col = route_record.col[route_record.cell_number - 1];
	}

	//��һ��·�ڵ����һ��·�ڣ���Ҫ���иĽ�A*�㷨������һ��ǰ������ 
	RouteRecord cross_record = CreateCrossRoute(p_row, p_col, must_cross_row, must_cross_col);   //�õ����辭����·�� 
	if (cross_record.cell_number > 1)
	{
		int direction;
		for (int i = 0; i < cross_record.cell_number - 1; i++)
		{
			if (cross_record.col[i] == cross_record.col[i + 1])   //��һ��·�ں���һ��·�����ڵ������ 
			{
				if (cross_record.row[i] > cross_record.row[i + 1])
					direction = UP;
				else
					direction = DOWN;
			}
			else        //��һ��·�ں���һ��·�����ڵ��в���ȣ�Ҳ��������� 
			{
				if (cross_record.col[i] > cross_record.col[i + 1])
					direction = LEFT;
				else
					direction = RIGHT;
			}
			if (direction == UP || direction == DOWN)
			{
				for (int j = 0;j < (WORKBENCH_LENGTH + 1);j++)
					route_record = AddIntoRouteRecord(route_record, direction);
			}
			else
			{
				for (int j = 0;j < (WORKBENCH_WIDTH + 1);j++)
					route_record = AddIntoRouteRecord(route_record, direction);
			}
		}
	}
	p_row = must_cross_row;
	p_col = must_cross_col;

	//�����һ���ؾ�·�ڵ����յ� 
	while ((p_row != task.get_end_row()) || (p_col != task.get_end_col()))
	{
		if (task.get_end_row() < p_row)
			direction = UP;
		if (task.get_end_row() > p_row)
			direction = DOWN;
		if (task.get_end_col() < p_col)
			direction = LEFT;
		if (task.get_end_col() > p_col)
			direction = RIGHT;

		route_record = AddIntoRouteRecord(route_record, direction);
		p_row = route_record.row[route_record.cell_number - 1];       //����������λ�� 
		p_col = route_record.col[route_record.cell_number - 1];
	}

	return AddTurnActionToRouteRecord(route_record);  //��·����ת��·�ڣ�����ʵ�ʵ�ת�䶯��
}

// �Ľ�A*·���滮�㷨������·�ڽڵ���ӽ�·������
RouteRecord RoutePlan::AddIntoRouteRecord(RouteRecord route_record, int direction)
{
	switch (direction)
	{
	case UP:          //������һ�� 
		if (route_record.row[route_record.cell_number - 1] > 0   //���ܳ����߽�&&����ײ�����
			&& cell[route_record.row[route_record.cell_number - 1] - 1][route_record.col[route_record.cell_number - 1]].kind != WORKBENCH)
		{
			route_record.row[route_record.cell_number] = route_record.row[route_record.cell_number - 1] - 1;
			route_record.col[route_record.cell_number] = route_record.col[route_record.cell_number - 1];
			route_record.cell_number++;
			break;
		}
		else
			break;

	case DOWN:          //������һ�� 
		if (route_record.row[route_record.cell_number - 1] < (ROWS - 1)
			&& cell[route_record.row[route_record.cell_number - 1] + 1][route_record.col[route_record.cell_number - 1]].kind != WORKBENCH)
		{
			route_record.row[route_record.cell_number] = route_record.row[route_record.cell_number - 1] + 1;
			route_record.col[route_record.cell_number] = route_record.col[route_record.cell_number - 1];
			route_record.cell_number++;
			break;
		}
		else
			break;

	case LEFT:          //������һ�� 
		if (route_record.col[route_record.cell_number - 1] > 0
			&& cell[route_record.row[route_record.cell_number - 1]][route_record.col[route_record.cell_number - 1] - 1].kind != WORKBENCH)
		{
			route_record.row[route_record.cell_number] = route_record.row[route_record.cell_number - 1];
			route_record.col[route_record.cell_number] = route_record.col[route_record.cell_number - 1] - 1;
			route_record.cell_number++;
			break;
		}
		else
			break;

	case RIGHT:          //������һ�� 
		if (route_record.col[route_record.cell_number - 1] < (COLS - 1)
			&& cell[route_record.row[route_record.cell_number - 1]][route_record.col[route_record.cell_number - 1] + 1].kind != WORKBENCH)
		{
			route_record.row[route_record.cell_number] = route_record.row[route_record.cell_number - 1];
			route_record.col[route_record.cell_number] = route_record.col[route_record.cell_number - 1] + 1;
			route_record.cell_number++;
			break;
		}
		else
			break;
	}
	return route_record;
}

// �Ľ�A*·���滮�㷨�����ɴ����·�ڵ��յ�·�ڵ�·������
RouteRecord RoutePlan::CreateCrossRoute(int beginrow, int begincol, int endrow, int endcol)
{
	if (cell[beginrow][begincol].kind != CROSS || cell[endrow][endcol].kind != CROSS)
		cout << "Not Cross" << endl;

	int endflag;
	openlist[0].row = beginrow;   //�����·�ڳ�ʼ��������open�� 
	openlist[0].col = begincol;
	openlist[0].gn = 0;
	openlist[0].hn = abs(endrow - beginrow) + abs(endcol - begincol);
	openlist[0].fn = openlist[0].gn + openlist[0].hn;
	openlist[0].inopen = 1;
	openlist[0].parent = 0;

	int len = 1;  //openlistlength     openlist���м���Ԫ�أ�len�͵��ڼ� 
	int clen = 0;   //closelistlength      closelist���м���Ԫ�أ�clen�͵��ڼ� 

	while (1)
	{
		/************�ҵ�openlist��inopen==1����fn��С��Ԫ��*************/
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

		openlist[mintag].inopen = 0;   //��fn��С�ĵ��Ƶ�closelist ��ʵ������㻹��openlist�У�ֻ��inopen����Ϊ0 
		closelist[clen] = openlist[mintag];
		clen++;

		if (openlist[mintag].row == endrow && openlist[mintag].col == endcol)  //���fn��С�ĵ����յ㣬Ѱ·���� 
		{
			endflag = mintag;
			break;
		}

		//�����ҷ����ϵĿɴ�ڵ����openlist 
		if (GetNextCrossLeftRight(openlist[mintag].row, openlist[mintag].col) != 0)
		{
			int p_row = openlist[mintag].row;
			int p_col = openlist[mintag].col + GetNextCrossLeftRight(openlist[mintag].row, openlist[mintag].col);
			int closeflag = 0;  //��ǰ�ڵ��ڲ���closelist�е�������0�����ڣ�1������ 
			int openflag = 0;
			int num;    //��ǰ�ڵ���open���У���¼�����ڵ�λ�� 

			for (int i = 0; i < clen; i++)   //�жϵ�ǰ�ڵ��ڲ���closelist�� 
			{
				if (closelist[i].row == p_row && closelist[i].col == p_col)
					closeflag = 1;
			}

			if (closeflag == 0)
			{
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
					openlist[len].gn = openlist[mintag].gn + (WORKBENCH_WIDTH + 1);
					if (mintag >= 1)     //���mintag>=1,Ҳ����mintag���ڸ��ڵ㣬�Ϳ����ж��Ƿ���turnvalue 
						openlist[len].gn = openlist[len].gn
						+ JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col, openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						+ road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					openlist[len].hn = abs(endrow - p_row) + abs(endcol - p_col);
					openlist[len].fn = openlist[len].gn + openlist[len].hn;
					openlist[len].inopen = 1;
					openlist[len].parent = mintag;
					len++;
				}
				else
				{                 //�����openlist�У��ͱȽ�gn����ǰgn��С�͸��ĸ�ĸ 
					int g = openlist[mintag].gn + (WORKBENCH_WIDTH + 1);
					if (mintag >= 1)     //���mintag>=1,Ҳ����mintag���ڸ��ڵ㣬�Ϳ����ж��Ƿ���turnvalue 
						g = g + JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col,openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						      + road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					if (g < openlist[num].gn)
						openlist[num].parent = mintag;
				}
			}
		}

		//�����·����ϵĿɴ�ڵ����openlist 
		if (GetNextCrossUpDown(openlist[mintag].row, openlist[mintag].col) != 0)
		{
			int p_row = openlist[mintag].row + GetNextCrossUpDown(openlist[mintag].row, openlist[mintag].col);
			int p_col = openlist[mintag].col;
			int closeflag = 0;  //��ǰ�ڵ��ڲ���closelist�е�������0�����ڣ�1������ 
			int openflag = 0;
			int num;    //��ǰ�ڵ���open���У���¼�����ڵ�λ�� 

			for (int i = 0; i < clen; i++)   //�жϵ�ǰ�ڵ��ڲ���closelist�� 
			{
				if (closelist[i].row == p_row && closelist[i].col == p_col)
					closeflag = 1;
			}

			if (closeflag == 0)
			{
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
					openlist[len].gn = openlist[mintag].gn + (WORKBENCH_LENGTH + 1);
					if (mintag >= 1)     //���mintag>=1,Ҳ����mintag���ڸ��ڵ㣬�Ϳ����ж��Ƿ���turnvalue 
						openlist[len].gn = openlist[len].gn
						+ JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col, openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						+ road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					openlist[len].hn = abs(endrow - p_row) + abs(endcol - p_col);
					openlist[len].fn = openlist[len].gn + openlist[len].hn;
					openlist[len].inopen = 1;
					openlist[len].parent = mintag;
					len++;
				}
				else
				{                 //�����openlist�У��ͱȽ�gn����ǰgn��С�͸��ĸ�ĸ 
					int g = openlist[mintag].gn + (WORKBENCH_LENGTH + 1);
					if (mintag >= 1)     //���mintag>=1,Ҳ����mintag���ڸ��ڵ㣬�Ϳ����ж��Ƿ���turnvalue 
						g = g + JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col, openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						      + road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					if (g < openlist[num].gn)
						openlist[num].parent = mintag;
				}
			}
		}
	}

	//���յ㿪ʼ��Ҳ����openlist[endflag],�����¼������·�ڵ�·������¼��record�� 
	RouteRecord record;
	int temp = endflag;
	record.cell_number = 0;
	while (1)
	{
		if (openlist[temp].row == beginrow && openlist[temp].col == begincol)
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
	return reall_record;
}

// �Ľ�A*·���滮�㷨���ж������Ƿ��ߣ�����ת�����
int RoutePlan::JudgeTurnValue(int last_row, int last_col, int pre_row, int pre_col, int next_row, int next_col)
{
	if (last_row == pre_row && pre_row == next_row || last_col == pre_col && pre_col == next_col)
		return 0;
	else
		return TURN_COST;
}

// �Ľ�A*·���滮�㷨��ȷ�����ҷ���ķ���
int RoutePlan::GetNextCrossLeftRight(int row, int col)
{
	int left_right;
	if (col == 0)   //ȷ�����ҷ���ķ��� 
		left_right = cell[row][col + 1].kind;
	else
		left_right = cell[row][col - 1].kind;
	if (left_right == LEFT_ROAD && col != 0)
		left_right = -(WORKBENCH_WIDTH + 1);
	else
	{
		if (left_right == RIGHT_ROAD && col != COLS - 1)
			left_right = (WORKBENCH_WIDTH + 1);
		else
			left_right = 0;
	}
	return left_right;
}

// �Ľ�A*·���滮�㷨��ȷ�����·���ķ���
int RoutePlan::GetNextCrossUpDown(int row, int col)
{
	int up_down;
	if (row == 0)    //ȷ�����·���ķ��� 
		up_down = cell[row + 1][col].kind;
	else
		up_down = cell[row - 1][col].kind;
	if (up_down == UP_ROAD && row != 0)   //�����ϵĲ��Ҳ���������һ�� 
		up_down = -(WORKBENCH_LENGTH + 1);
	else
	{
		if (up_down == DOWN_ROAD && row != ROWS - 1)  //�����µĲ��Ҳ���������һ��
			up_down = (WORKBENCH_LENGTH + 1);
		else
			up_down = 0;     //��������Ҳ�������� 
	}
	return up_down;
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