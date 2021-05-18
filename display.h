#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <windows.h>

class Display
{
public:
    void HiddenCursor();  //隐藏光标    
    void GotoXY(int x, int y);  //移动光标 
    void SetColor(int color);  //设置输出的图形的颜色
    void SetInterface();  //设置界面参数
};

extern Display display;
#endif // !_DISPLAY_H