#include "display.h"

Display display;

// ���ع��
void Display::HiddenCursor()
{
	HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
	CONSOLE_CURSOR_INFO cci;
	GetConsoleCursorInfo(hOut, &cci);
	cci.bVisible = 0;//��1Ϊ��ʾ����0Ϊ����
	SetConsoleCursorInfo(hOut, &cci);
}

// �ƶ���� 
void Display::GotoXY(int x, int y)
{
	COORD coord;
	coord.X = y;
	coord.Y = x;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
}

// �������ͼ�ε���ɫ
void Display::SetColor(int color)
{
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	if (hConsole != INVALID_HANDLE_VALUE)
	{
		SetConsoleTextAttribute(hConsole, color);
	}
}

// ����չʾ����Ĳ���
void Display::SetInterface()
{
	
	system("mode con COLS=200 lines=55");
	system("color F8");
	system("title run_test");
	display.HiddenCursor();
}