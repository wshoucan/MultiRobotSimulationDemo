#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <windows.h>

class Display
{
public:
    void HiddenCursor();  //���ع��    
    void GotoXY(int x, int y);  //�ƶ���� 
    void SetColor(int color);  //���������ͼ�ε���ɫ
    void SetInterface();  //���ý������
};

extern Display display;
#endif // !_DISPLAY_H