#include "display.h"

Display display;

// 隐藏光标
void Display::HiddenCursor()
{
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cci;
	GetConsoleCursorInfo(hOut, &cci);
	cci.bVisible = 0;//赋1为显示，赋0为隐藏
	SetConsoleCursorInfo(hOut, &cci);
}

// 移动光标 
void Display::GotoXY(int x, int y)
{
	COORD coord;
	coord.X = y;
	coord.Y = x;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

// 设置输出图形的颜色
void Display::SetColor(int color)
{
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hConsole != INVALID_HANDLE_VALUE)
	{
		SetConsoleTextAttribute(hConsole, color);
	}
}

// 设置展示界面的参数
void Display::SetInterface()
{
	
	system("mode con COLS=200 lines=55");
	system("color F8");
	system("title run_test");
	display.HiddenCursor();
}