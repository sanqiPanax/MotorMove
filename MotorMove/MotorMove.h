#pragma once
#include "motormove_global.h"
#include <iostream>
#include "USB1020.h"
#include<thread>

#define XAXIS 0b00000001
#define YAXIS 0b00000010
#define ZAXIS 0b00000100

#define FORWARD 1
#define BACK 2
#define SETZERO 3
#define BACKZERO 4


using namespace std;

extern "C" class MOTORMOVE_EXPORT MotorMove : public QObject
{
    Q_OBJECT
public:
    MotorMove(QObject* parent=Q_NULLPTR);
    ~MotorMove();

    // (形参：哪些轴（二进制表示），执行功能，脉冲数)基础运动，负责电机的基本运动功能，包括三个轴的前进后退，置零，复位
    void basedMove(int axis,int function,int pulse);
    // （形参：步数（4步是3个位置，8步是5个位置，12步是7个位置),步距（每步多长）），
    void zAxisLoopMove(int steps,int step_distance);
    // （形参：矩阵的长宽（每行多少孔，每列多少个孔），水平孔距（两个相邻孔之间的距离），垂直孔距） xy插补运动，
    void xyAxisMove(int length,int width,int level_distance,int vertical_distance);

signals:
    void basedMoveComplate(unsigned int);//基础运动完成信号；将轴二进制数发回
    void zAxisLoopMoveComplate(int);//形参就是第多少次（或者说第多少步）
    void xyAxisMoveComplate(int, int);//形参就是当前位置
private:
    //设备，公共参数结构体等
    HANDLE hDevice;
    
    USB1020_PARA_DataList DL;
    
    USB1020_PARA_LCData LC;
    
    USB1020_PARA_LineData LD;//两轴和三轴需要的变量

    USB1020_PARA_InterpolationAxis IA;

    /////////////////////////自定义的私有变量////////////////////////////////
    int x_axis = 0;
    int y_axis = 0;
    int z_axis = 0;
    int nums_of_axis = 0;


    /////////////////////////////////////////////////////////

    //以下为私有函数，由公有函数调用

    //1.功能性函数，前进后退等
    //前进 function设为1
    void moveForward(int pulse);
    //后退 function设为2
    void moveBack(int pulse);
    //将当前位置设置为零点 function设为3
    void setZero();
    //回到原点 function设为4
    void backToZero();
    //z轴的往返运动
    void zAxisCircularMotion();
    //xy轴弓字型运动
    void xyAxisArchMotion();

    //2.选轴
    
    //选轴
    void decadeAxis(int axis);
    //轴置零
    void resetAxis();

    //单轴
    void singleAxis();

    void doubleAxis();

    void triAxis();

    //3.输出信息
    //输出当前位置
    void showCurrentLocation();

    //基础运动函数完成信号发送函数，线程
    void basedThreadSend();

    //z轴的信号发送和线程处理
    void zAxisThreadSend(int output);

    //xy轴“弓”型运动的信号发送和线程处理
    void xyAxisThreadSend(int location_x, int location_y);
};
