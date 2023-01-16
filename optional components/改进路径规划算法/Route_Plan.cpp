#include "Route_Plan.h"

using namespace std;

// 根据路径规划的种类，选取对应的算法，生成路径
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

// 标准A*路径规划算法
RouteRecord RoutePlan::AStarPathPlanning(Task task)
{
	openlist[0].row = task.get_begin_row();   //将起点初始化，加入open表 
	openlist[0].col = task.get_begin_col();
	openlist[0].gn = 0;
	openlist[0].hn = abs(task.get_end_row() - task.get_begin_row()) + abs(task.get_end_col() - task.get_begin_col());
	openlist[0].fn = openlist[0].gn + openlist[0].hn;
	openlist[0].inopen = 1;
	openlist[0].parent = 0;

	int endflag;
	int len = 1;  //openlistlength     openlist中有几个元素，len就等于几 
	int clen = 0;   //closelistlength      closelist中有几个元素，clen就等于几 

	while (1)
	{
		// 找到openlist中inopen==1并且fn最小的元素
		int minfn;
		int mintag = 0;
		while (1)    //在openlist表中找到第一个inopen==1的元素 
		{
			if (openlist[mintag].inopen == 0)
				mintag++;
			else
			{
				minfn = openlist[mintag].fn;
				break;
			}
		}
		for (int i = mintag; i < len; i++)   //找到总代价fn最小的元素，元素在openlist表中的位置为mintag 
		{
			if (openlist[i].inopen == 1 && openlist[i].fn < minfn)
			{
				mintag = i;
				minfn = openlist[i].fn;
			}
		}

		if (openlist[mintag].row == task.get_end_row() && openlist[mintag].col == task.get_end_col())  //如果fn最小的点是终点，寻路结束 
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

		openlist[mintag].inopen = 0;   //将fn最小的点移到closelist ，实际这个点还在openlist中，只是inopen被置为0 
		closelist[clen] = openlist[mintag];
		clen++;
	}

	// 从终点开始，也就是openlist[endflag],依次查找父节点，倒序记录经过的路口的路径，记录到record中 
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

	//顺序记录经过的路口，也就是把record倒过来，变成reall_record 
	RouteRecord reall_record;
	reall_record.cell_number = 0;
	for (int i = record.cell_number - 1; i >= 0; i--)
	{
		reall_record.row[reall_record.cell_number] = record.row[i];
		reall_record.col[reall_record.cell_number] = record.col[i];
		reall_record.cell_number++;
	}

	return AddTurnActionToRouteRecord(reall_record);   //在路径的转弯路口，增加实际的转弯动作
}

// 标准A*路径规划算法：将节点添加进OpenList
bool RoutePlan::AStarPushIntoOpenlist(Task task, int mintag, int len, int clen, int p_row, int p_col)  //此处的p_row和p_col是等待放入openlist中的 
{
	int closeflag = 0;  //当前节点在不在closelist中的象征，0代表不在，1代表在 
	int openflag = 0;
	int num;    //当前节点在open表中，记录它所在的位置 

	for (int i = 0; i < len; i++)   //判断当前节点在不在closelist中 
	{
		if (openlist[i].row == p_row && openlist[i].col == p_col && openlist[i].inopen == 0)
		{
			closeflag = 1;
			return false;
		}
	}

	for (int i = 0; i < len; i++)    //判断当前节点在不在openlist中 
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
		openlist[len].row = p_row;   //如果不在open表中，就把当前节点加入openlist 
		openlist[len].col = p_col;
		openlist[len].gn = openlist[mintag].gn + 1;
		openlist[len].hn = abs(task.get_end_row() - p_row) + abs(task.get_end_col() - p_col);
		openlist[len].fn = openlist[len].gn + openlist[len].hn;
		openlist[len].inopen = 1;
		openlist[len].parent = mintag;
		return true;    //open表中添加了新元素。 
	}
	else
	{                 //如果在openlist中，就比较gn，当前gn较小就更改父母 
		if (openlist[mintag].gn + 1 < openlist[num].gn)
		{
			openlist[num].parent = mintag;
			return false;   //open表中没有添加新元素 
		}
	}
}

// 标准A*路径规划算法：判断路口的种类（即路口的上下左右方向是否允许通行）
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



// 改进A*路径规划算法
RouteRecord RoutePlan::ImprovedAStarPathPlanning(Task task)
{
	//cout << task.get_begin_row() << "  " << task.get_begin_col() << "  " << task.get_end_row() << "  " << task.get_end_col() << "  " << endl;
	RouteRecord route_record;    //记录机器人经过的路线 
	route_record.row[0] = task.get_begin_row();
	route_record.col[0] = task.get_begin_col();
	route_record.cell_number = 1;  //机器人所走的步数 

	int must_cross_row;   //想要到达终点而必须经过的路口的行 
	int must_cross_col = task.get_end_col();  //想要到达终点而必须经过的路口的列 
	if (cell[task.get_end_row()][task.get_end_col()].kind == UP_ROAD)    //分为上行路口与下行路口 
		must_cross_row = ((int)(task.get_end_row() / (WORKBENCH_LENGTH + 1) + 1)) * (WORKBENCH_LENGTH + 1);
	else
		must_cross_row = ((int)(task.get_end_row() / (WORKBENCH_LENGTH + 1))) * (WORKBENCH_LENGTH + 1);

	int direction;  //用来指示下一步所走的方向
	int p_row = route_record.row[0];       //现在所处的位置 
	int p_col = route_record.col[0];

    //起点与终点在同一条竖线上，并且方向可达
	if ((int)(task.get_begin_row() / (WORKBENCH_LENGTH + 1)) == (int)(task.get_end_row() / (WORKBENCH_LENGTH + 1)) && task.get_begin_col() == task.get_end_col() //这个条件只能说明在同一小区域的竖线上 
		&& ((task.get_end_row() - task.get_begin_row() <= 0 && cell[p_row][p_col].kind == UP_ROAD)
			|| (task.get_end_row() - task.get_begin_row() >= 0 && cell[p_row][p_col].kind == DOWN_ROAD)))
	{
		if (task.get_end_row() - task.get_begin_row() <= 0 && cell[p_row][p_col].kind == UP_ROAD)  //起点在终点上面 
			direction = UP;
		else
			direction = DOWN;

		while ((p_row != task.get_end_row()) || (p_col != task.get_end_col()))  //判断是否到达终点 
		{
			route_record = AddIntoRouteRecord(route_record, direction);
			p_row = route_record.row[route_record.cell_number - 1];
			p_col = route_record.col[route_record.cell_number - 1];
		}
		return route_record;
	}

	//起点与终点在同一条横线上，并且方向可达
	if ((int)(task.get_begin_col() / (WORKBENCH_WIDTH + 1)) == (int)(task.get_end_col() / (WORKBENCH_WIDTH + 1)) && task.get_begin_row() == task.get_end_row() //这个条件只能说明在同一小区域的竖线上 
		&& ((task.get_end_col() - task.get_begin_col() <= 0 && cell[p_row][p_col].kind == LEFT_ROAD)
			|| (task.get_end_col() - task.get_begin_col() >= 0 && cell[p_row][p_col].kind == RIGHT_ROAD)))
	{
		if (task.get_end_col() - task.get_begin_col() <= 0 && cell[p_row][p_col].kind == LEFT_ROAD)  //起点在终点上面 
			direction = LEFT;
		else
			direction = RIGHT;

		while ((p_row != task.get_end_row()) || (p_col != task.get_end_col()))  //判断是否到达终点 
		{
			route_record = AddIntoRouteRecord(route_record, direction);
			p_row = route_record.row[route_record.cell_number - 1];
			p_col = route_record.col[route_record.cell_number - 1];
		}
		return route_record;
	}

    // 起点与终点不在同一小区域

    //到达第一个路口 
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

	//第一个路口到最后一个路口，需要运行改进A*算法决定下一步前进方向 
	RouteRecord cross_record = CreateCrossRoute(p_row, p_col, must_cross_row, must_cross_col);   //得到所需经过的路口 
	if (cross_record.cell_number > 1)
	{
		int direction;
		for (int i = 0; i < cross_record.cell_number - 1; i++)
		{
			if (cross_record.col[i] == cross_record.col[i + 1])   //这一个路口和下一个路口所在的列相等 
			{
				if (cross_record.row[i] > cross_record.row[i + 1])
					direction = UP;
				else
					direction = DOWN;
			}
			else        //这一个路口和下一个路口所在的列不相等，也就是行相等 
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

	//从最后一个必经路口到达终点 
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
		p_row = route_record.row[route_record.cell_number - 1];       //现在所处的位置 
		p_col = route_record.col[route_record.cell_number - 1];
	}

	return AddTurnActionToRouteRecord(route_record);  //在路径的转弯路口，增加实际的转弯动作
}

// 改进A*路径规划算法：将非路口节点添加进路径序列
RouteRecord RoutePlan::AddIntoRouteRecord(RouteRecord route_record, int direction)
{
	switch (direction)
	{
	case UP:          //向上走一步 
		if (route_record.row[route_record.cell_number - 1] > 0   //不能超过边界&&不能撞向货架
			&& cell[route_record.row[route_record.cell_number - 1] - 1][route_record.col[route_record.cell_number - 1]].kind != WORKBENCH)
		{
			route_record.row[route_record.cell_number] = route_record.row[route_record.cell_number - 1] - 1;
			route_record.col[route_record.cell_number] = route_record.col[route_record.cell_number - 1];
			route_record.cell_number++;
			break;
		}
		else
			break;

	case DOWN:          //向下走一步 
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

	case LEFT:          //向左走一步 
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

	case RIGHT:          //向右走一步 
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

// 改进A*路径规划算法：生成从起点路口到终点路口的路口序列
RouteRecord RoutePlan::CreateCrossRoute(int beginrow, int begincol, int endrow, int endcol)
{
	if (cell[beginrow][begincol].kind != CROSS || cell[endrow][endcol].kind != CROSS)
		cout << "Not Cross" << endl;

	int endflag;
	openlist[0].row = beginrow;   //将起点路口初始化，加入open表 
	openlist[0].col = begincol;
	openlist[0].gn = 0;
	openlist[0].hn = abs(endrow - beginrow) + abs(endcol - begincol);
	openlist[0].fn = openlist[0].gn + openlist[0].hn;
	openlist[0].inopen = 1;
	openlist[0].parent = 0;

	int len = 1;  //openlistlength     openlist中有几个元素，len就等于几 
	int clen = 0;   //closelistlength      closelist中有几个元素，clen就等于几 

	while (1)
	{
		/************找到openlist中inopen==1并且fn最小的元素*************/
		int minfn;
		int mintag = 0;
		while (1)    //在openlist表中找到第一个inopen==1的元素 
		{
			if (openlist[mintag].inopen == 0)
				mintag++;
			else
			{
				minfn = openlist[mintag].fn;
				break;
			}
		}
		for (int i = mintag; i < len; i++)   //找到总代价fn最小的元素，元素在openlist表中的位置为mintag 
		{
			if (openlist[i].inopen == 1 && openlist[i].fn < minfn)
			{
				mintag = i;
				minfn = openlist[i].fn;
			}
		}

		openlist[mintag].inopen = 0;   //将fn最小的点移到closelist ，实际这个点还在openlist中，只是inopen被置为0 
		closelist[clen] = openlist[mintag];
		clen++;

		if (openlist[mintag].row == endrow && openlist[mintag].col == endcol)  //如果fn最小的点是终点，寻路结束 
		{
			endflag = mintag;
			break;
		}

		//将左右方向上的可达节点加入openlist 
		if (GetNextCrossLeftRight(openlist[mintag].row, openlist[mintag].col) != 0)
		{
			int p_row = openlist[mintag].row;
			int p_col = openlist[mintag].col + GetNextCrossLeftRight(openlist[mintag].row, openlist[mintag].col);
			int closeflag = 0;  //当前节点在不在closelist中的象征，0代表不在，1代表在 
			int openflag = 0;
			int num;    //当前节点在open表中，记录它所在的位置 

			for (int i = 0; i < clen; i++)   //判断当前节点在不在closelist中 
			{
				if (closelist[i].row == p_row && closelist[i].col == p_col)
					closeflag = 1;
			}

			if (closeflag == 0)
			{
				for (int i = 0; i < len; i++)    //判断当前节点在不在openlist中 
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
					openlist[len].row = p_row;   //如果不在open表中，就把当前节点加入openlist 
					openlist[len].col = p_col;
					openlist[len].gn = openlist[mintag].gn + (WORKBENCH_WIDTH + 1);
					if (mintag >= 1)     //如果mintag>=1,也就是mintag存在父节点，就可以判断是否有turnvalue 
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
				{                 //如果在openlist中，就比较gn，当前gn较小就更改父母 
					int g = openlist[mintag].gn + (WORKBENCH_WIDTH + 1);
					if (mintag >= 1)     //如果mintag>=1,也就是mintag存在父节点，就可以判断是否有turnvalue 
						g = g + JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col,openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						      + road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					if (g < openlist[num].gn)
						openlist[num].parent = mintag;
				}
			}
		}

		//将上下方向上的可达节点加入openlist 
		if (GetNextCrossUpDown(openlist[mintag].row, openlist[mintag].col) != 0)
		{
			int p_row = openlist[mintag].row + GetNextCrossUpDown(openlist[mintag].row, openlist[mintag].col);
			int p_col = openlist[mintag].col;
			int closeflag = 0;  //当前节点在不在closelist中的象征，0代表不在，1代表在 
			int openflag = 0;
			int num;    //当前节点在open表中，记录它所在的位置 

			for (int i = 0; i < clen; i++)   //判断当前节点在不在closelist中 
			{
				if (closelist[i].row == p_row && closelist[i].col == p_col)
					closeflag = 1;
			}

			if (closeflag == 0)
			{
				for (int i = 0; i < len; i++)    //判断当前节点在不在openlist中 
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
					openlist[len].row = p_row;   //如果不在open表中，就把当前节点加入openlist 
					openlist[len].col = p_col;
					openlist[len].gn = openlist[mintag].gn + (WORKBENCH_LENGTH + 1);
					if (mintag >= 1)     //如果mintag>=1,也就是mintag存在父节点，就可以判断是否有turnvalue 
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
				{                 //如果在openlist中，就比较gn，当前gn较小就更改父母 
					int g = openlist[mintag].gn + (WORKBENCH_LENGTH + 1);
					if (mintag >= 1)     //如果mintag>=1,也就是mintag存在父节点，就可以判断是否有turnvalue 
						g = g + JudgeTurnValue(openlist[openlist[mintag].parent].row, openlist[openlist[mintag].parent].col, openlist[mintag].row, openlist[mintag].col, p_row, p_col)
						      + road_jam_status.GetnodeStatusCost(beginrow, begincol, p_row, p_col);
					if (g < openlist[num].gn)
						openlist[num].parent = mintag;
				}
			}
		}
	}

	//从终点开始，也就是openlist[endflag],倒序记录经过的路口的路径，记录到record中 
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
	//顺序记录经过的路口，也就是把record倒过来，变成reall_record 
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

// 改进A*路径规划算法：判断三点是否共线，返回转弯代价
int RoutePlan::JudgeTurnValue(int last_row, int last_col, int pre_row, int pre_col, int next_row, int next_col)
{
	if (last_row == pre_row && pre_row == next_row || last_col == pre_col && pre_col == next_col)
		return 0;
	else
		return TURN_COST;
}

// 改进A*路径规划算法：确定左右方向的方向
int RoutePlan::GetNextCrossLeftRight(int row, int col)
{
	int left_right;
	if (col == 0)   //确定左右方向的方向 
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

// 改进A*路径规划算法：确定上下方向的方向
int RoutePlan::GetNextCrossUpDown(int row, int col)
{
	int up_down;
	if (row == 0)    //确定上下方向的方向 
		up_down = cell[row + 1][col].kind;
	else
		up_down = cell[row - 1][col].kind;
	if (up_down == UP_ROAD && row != 0)   //是向上的并且不在最上面一行 
		up_down = -(WORKBENCH_LENGTH + 1);
	else
	{
		if (up_down == DOWN_ROAD && row != ROWS - 1)  //是向下的并且不在最下面一行
			up_down = (WORKBENCH_LENGTH + 1);
		else
			up_down = 0;     //不能向上也不能向下 
	}
	return up_down;
}

// 将实际的转弯动作添加进路径记录中，各种路径规划算法生成路径后都要调用本函数
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