#ifndef _DEFINE_H
#define _DEFINE_H

/***************定义工作场景**************/
#define WORKBENCH 1  //货架（工作台）
#define UP_ROAD 2   //上行通道
#define DOWN_ROAD 3   //下行通道
#define LEFT_ROAD 4   //左行通道
#define RIGHT_ROAD 5  //右行通道
#define CROSS 6  //路口

/***********设置工作场景的大小************/
#define WORKBENCH_WIDTH 2   //每个货架小组中，横向货架的个数 ，因为场景布局限制，这个数必须是2 
#define WORKBENCH_LENGTH 5    //每个货架小组中，竖向货架的个数，可以为任意个
#define WORKBENCH_WIDTH_NUMBER 9 //横向货架小组的个数 ,必须是奇数 
#define WORKBENCH_LENGTH_NUMBER 3  //纵向货架小组的个数 ，必须是奇数 
#define ROWS (WORKBENCH_LENGTH + 1) * WORKBENCH_LENGTH_NUMBER + 1  //场景总行数
#define COLS (WORKBENCH_WIDTH + 1) * WORKBENCH_WIDTH_NUMBER + 1 //场景总列数

/*************方向设定*****************/
#define UP 0   
#define DOWN 1
#define LEFT 2
#define RIGHT 3

/***************通道状态表**************/
#define ROAD_FREE 1   //通道空闲 
#define ROAD_RESERVED 2   //通道被预约 
#define ROAD_OCCUPIED 3  //通道被占用 

/***************机器人状态****************/
#define ROBOT_FREE 5  //机器人状态：空闲 
#define ROBOT_CARRYING 6   //机器人状态：正在执行搬运任务 
#define ROBOT_SCHEDULED 7   //机器人状态：正在从被从上一个任务的终点调度到下一个任务的起点
#define ROBOT_SCHEDULED_COMPLETED 10  //机器人状态：完成调度 

/****************显示工作场景***********************/
#define RED 252   //红色
#define GRAY 248   //灰色
#define GREEN 250  //绿色
#define YELLOW 246  //黄色

/*****************路径规划算法的选取********************/
#define A_STAR 1  //标准A*算法
#define PATH_PLANNING_ALGORITHM_TYPE 1   //所选择的路径规划算法类型

/***************几个可调参数**************/
#define TURN_COST 2  //转弯代价
#define MAX_TASK_NUM 5000   //可执行的任务总数
#define ROBOT_NUM 10  //机器人的个数

#endif // ! _DEFINE_H

