#include "main.h"

using namespace std;

int main()
{
	display.SetInterface();

	// ��ʼ����ͼ�������·״̬������״̬����������������ļ��ĳ�ʼ�����ڹ��캯�����Զ�������
	InitMap();
	DrawMap();

   // ����һ���������
	srand(3);   //srand((unsigned)time(NULL));
	Task *task = new Task[MAX_TASK_NUM];
	for (int i = 0; i < MAX_TASK_NUM; i++)     
		task[i] = task[i].CreateTask();

   // �����л����˽��г�ʼ����������Ļ�����
	for (int ID = 0; ID < ROBOT_NUM; ID++)    
	{
		robot[ID].InitRobot(ID);
	}

    int clock = 0;   //ʱ������
	int task_num = 0;  //����ִ�е��ڼ���task�� 
	int complete_task_num = 0;  //�Ѿ���ɵ�����ĸ���

    // ����һ������ˣ��ѵ�һ����������ÿһ�������� 
    for (int ID = 0; ID < ROBOT_NUM; ID++)    
    {
	    task_num = robot[ID].FirstTaskToRobot(task[task_num], task_num);
    }

    // ��ÿ�������˽��б�����������������ǰ�н�һ�� 
	while (1)    
	{

		clock++;
		//DrawRoadStatus();

		// �ж���һ��Ӧ����ô��
		for (int ID = 0; ID < ROBOT_NUM; ID++)
		{
			robot[ID].JudgeNextStep();
		}

		// ʵ���ƶ�һ���������ǰλ����·���յ㣬�ͽ��������״̬ת��
		for (int ID = 0; ID < ROBOT_NUM; ID++)
		{
            robot[ID].MoveSingleStep();
			if (robot[ID].get_next_step() == 1)
			{
				if (robot[ID].RobotStatusTransform(task[task_num]))
				{
					complete_task_num++;
             		task_num++;
				}
			}
		}
		Sleep(50);
	}
}