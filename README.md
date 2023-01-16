# MultiRobotSimulationDemo
功能：提供一个栅格式多机器人仿真运行环境，用于对多机器人路径规划算法与协同控制进行仿真实验  
编写语言：C++   
运行环境：Windows11, VS2019  

仿真场地示例：  
<img width="329" alt="image" src="https://user-images.githubusercontent.com/56618904/212604940-e1d23979-0fd6-4164-9423-c26fa1e43dd7.png">

源码文件中注释已较为完备，仅对各源码文件所实现的功能做简单阐述  
7个.h文件，6个.cpp文件，除define.h外，.h文件与.cpp文件一一对应  

（1）define.h: 定义多个可调整参数，如机器人个数，场景规模等  
（2）main.cpp: 程序的入口，初始化机器人，循环执行任务  
（3）workshop.cpp: 初始化多机器人工作场地  
（4）display.cpp: 在屏幕输出场景与机器人，提供展示效果  
（5）task.cpp: 根据场景信息，生成随机任务  
（6）route_plan.cpp: 内置A星算法作为默认路径规划算法  
（7）multi_robot.cpp: 机器人的移动与状态转换  

所以可调整参数均位于define.h文件中，此处对几个主要参数做简要说明，  
#define WORKBENCH_WIDTH 2   //每个货架小组中，横向货架的个数，可以为任意个  
#define WORKBENCH_LENGTH 5    //每个货架小组中，竖向货架的个数，可以为任意个  
#define WORKBENCH_WIDTH_NUMBER 9 //横向货架小组的个数 ,必须为奇数   
#define WORKBENCH_LENGTH_NUMBER 3  //纵向货架小组的个数 ，必为是奇数   
#define MAX_TASK_NUM 5000   //可执行的任务总数  
#define ROBOT_NUM 10  //机器人的个数  

说明：  
（1）为方便根据需要添加其他组件与功能，本项目仅提供基础路径规划算法与碰撞避免方法  
（2）本仿真场景采用栅格化的建模方式，不涉及机器人速度、加速度等信息  
