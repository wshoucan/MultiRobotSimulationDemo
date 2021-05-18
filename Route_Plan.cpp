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