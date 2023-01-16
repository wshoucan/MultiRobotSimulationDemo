#ifndef _MULTI_ROBOT_H
#define _MULTI_ROBOT_H

#include <iostream>
#include <windows.h>
#include <stdlib.h>   //��������� 
#include <time.h>    
#include <math.h> 

#include "Route_Plan.h"
#include "display.h"
#include "workshop.h"
#include "road_occupy_status.h"

class Robot
{
private:
	int ID;  //�����˵ı��
	int p_row;    //�����˵�ǰ����λ�õ��� 
	int p_col;    //�����˵�ǰ����λ�õ��� 
	int status;   //�����˵ĵ�ǰ״̬ 
	Task real_task;    //ʵ�ʵİ�������ÿ�λ�ȡ�µİ�������ʱ�Ÿ�ֵ
	Task task;     //�����˵�ǰ��ִ�е����� ,�����ǵ��ȣ�Ҳ�����ǰ��ˣ���route_record��Ӧ
	RoutePlan route_plan;  //ר��������·���滮�����ɶ���
	RouteRecord route_record;   //�����˵�ǰ���н�·�� 
	int p_num;     //�����˵�ǰִ�е��˱�����ĵڼ��� 
	int next_step_flag;  //��������һ��Ӧ����ô�ƶ���ָʾ

public:
	int get_next_step()
	{
		return next_step_flag;
	}
	int get_p_row()
	{
		return p_row;
	}
	int get_p_col()
	{
		return p_col;
	}

    Robot();  //���캯��
	void InitRobot(int _ID); //�Ի����˳�ʼ�� 
	int FirstTaskToRobot(Task task,int task_num);  //ϵͳ��ʼ���󣬸�ÿ�������˸���һ����ʼ����
    void TaskToRobot(Task);   //��һ��������һ�������� 
    void JudgeNextStep();   //�ж���һ��������
    void MoveSingleStep();    //�����ΪID�Ļ����˽���һ�β�������β�������next_step���� 
    int RobotStatusTransform(Task task);    //JudgeNextStep()��������ֵΪ1ʱ������״̬ת��
};

extern Robot* robot;   //���ɶ����������� 

#endif