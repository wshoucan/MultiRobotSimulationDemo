#ifndef _DEFINE_H
#define _DEFINE_H

/***************���幤������**************/
#define WORKBENCH 1  //���ܣ�����̨��
#define UP_ROAD 2   //����ͨ��
#define DOWN_ROAD 3   //����ͨ��
#define LEFT_ROAD 4   //����ͨ��
#define RIGHT_ROAD 5  //����ͨ��
#define CROSS 6  //·��

/***********���ù��������Ĵ�С************/
#define WORKBENCH_WIDTH 2   //ÿ������С���У�������ܵĸ��� ����Ϊ�����������ƣ������������2 
#define WORKBENCH_LENGTH 5    //ÿ������С���У�������ܵĸ���������Ϊ�����
#define WORKBENCH_WIDTH_NUMBER 9 //�������С��ĸ��� ,���������� 
#define WORKBENCH_LENGTH_NUMBER 3  //�������С��ĸ��� ������������ 
#define ROWS (WORKBENCH_LENGTH + 1) * WORKBENCH_LENGTH_NUMBER + 1  //����������
#define COLS (WORKBENCH_WIDTH + 1) * WORKBENCH_WIDTH_NUMBER + 1 //����������

/*************�����趨*****************/
#define UP 0   
#define DOWN 1
#define LEFT 2
#define RIGHT 3

/***************ͨ��״̬��**************/
#define ROAD_FREE 1   //ͨ������ 
#define ROAD_RESERVED 2   //ͨ����ԤԼ 
#define ROAD_OCCUPIED 3  //ͨ����ռ�� 

/***************������״̬****************/
#define ROBOT_FREE 5  //������״̬������ 
#define ROBOT_CARRYING 6   //������״̬������ִ�а������� 
#define ROBOT_SCHEDULED 7   //������״̬�����ڴӱ�����һ��������յ���ȵ���һ����������
#define ROBOT_SCHEDULED_COMPLETED 10  //������״̬����ɵ��� 

/****************��ʾ��������***********************/
#define RED 252   //��ɫ
#define GRAY 248   //��ɫ
#define GREEN 250  //��ɫ
#define YELLOW 246  //��ɫ

/*****************·���滮�㷨��ѡȡ********************/
#define A_STAR 1  //��׼A*�㷨
#define PATH_PLANNING_ALGORITHM_TYPE 1   //��ѡ���·���滮�㷨����

/***************�����ɵ�����**************/
#define TURN_COST 2  //ת�����
#define MAX_TASK_NUM 5000   //��ִ�е���������
#define ROBOT_NUM 10  //�����˵ĸ���

#endif // ! _DEFINE_H

