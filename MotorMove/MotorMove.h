#ifndef _MOTOR_MOVE_H_
#define _MOTOR_MOVE_H_

#include "motormove_global.h"
#include <iostream>
#include "USB1020.h"
#include <thread>
#include <QDebug>

#define XAXIS 0b00000001
#define YAXIS 0b00000010
#define ZAXIS 0b00000100
#define XYAXIS 0b00000011
#define XZAXIS 0b00000101
#define YZAXIS 0b00000110
#define XYZAXIS 0b00000111

#define FORWARD 1
#define BACK 2
#define SETZERO 3
#define BACKZERO 4

extern "C" class MOTORMOVE_API MotorMove : public QObject
{
    Q_OBJECT
public:

    MotorMove(QObject* parent = Q_NULLPTR);
    ~MotorMove();


    // (形参：哪些轴（二进制表示），执行功能，脉冲数)基础运动，负责电机的基本运动功能，包括三个轴的前进后退，置零，复位
    void basedMove(int axis, int function, int pulse);
    // （形参：步数（4步是3个位置，8步是5个位置，12步是7个位置),步距（每步多长）），
    void zAxisLoopMove(int steps, int step_distance);
    // （形参：矩阵的长宽（每行多少孔，每列多少个孔），水平孔距（两个相邻孔之间的距离），垂直孔距） xy插补运动，
    void xyAxisMove(int length, int width, double level_distance, double vertical_distance);
    //z轴的信号发送和线程处理
    void zAxisThreadSend();
    //xy轴的信号发送和线程处理
    //xy轴“弓”型运动的信号发送和线程处理
    void xyAxisThreadSend();
    //输出当前位置
    void showCurrentLocation();

    //移动z轴，走到最清晰的点
    //void moveToMostClearPoint(int large_range,int small_range);
    void moveToMostClearPoint(double et_threshold, int pulseDistance);

    //让xy移动到边界，和“弓”型运动的方向相反 
    void moveToBorder(double x_axis_back,double y_axis_back);

signals:
    void basedMoveComplate();//基础运动完成信号；将轴二进制数发回
    void zAxisLoopMoveComplate();//形参就是第多少次（或者说第多少步）
    void xyAxisMoveComplate();//形参就是当前位置
    void startMeasure();
    void createError();
    void getMidCameraMtf(double& inp_mtf);

    void arrivedBounryBegin();//到达边界发送的信号
    void arrivedBounryEnd();

    void updateValue();
    void changeChearPointValue(double mtf_value);
    void showNowLocation(double xAxisLocation,double yAxisLocation,double zAxisLocation);//发送当前位置的信号

    
public slots:
    void bounryMove(double journey);//在两个边界之间移动,要在获取了当前位置调用

    void getClearValue(double value);//获得当前清晰度的函数，和对方的信号连接

    void etStop();//中断自动调焦
private:
    //设备，公共参数结构体等
    HANDLE hDevice;

    USB1020_PARA_DataList DL;

    USB1020_PARA_LCData LC;

    USB1020_PARA_LineData LD;//两轴和三轴需要的变量

    USB1020_PARA_InterpolationAxis IA;

    //用指针声明对象
    USB1020_PARA_DataList* DL1 = new USB1020_PARA_DataList();
    USB1020_PARA_LCData* LC1 = new USB1020_PARA_LCData();
    USB1020_PARA_LineData* LD1 = new USB1020_PARA_LineData();
    ////////////////////自定义的私有变量////////////////
    int x_axis = 0;
    int y_axis = 0;
    int z_axis = 0;
    int nums_of_axis = 0;

    ////////////////z轴使用////////////////////////////
    int zAxis_pulse = 0;
    int zAxis_steps = 0;//一节
    int zAxis_half_steps = 0;//一半
    ////////////////xy轴使用///////////////////////////
    int xLength = 0;
    int yWidth = 0;
    int xDistance = 0;
    int yDistance = 0;
    int xyTimes = 1;
    int yCount = 0;//用于切换方向
    /////////////////////////////////////////////////////////
    double clear_value = 0;
    double now_location = 0;//现在的位置
    ///////////////自动调焦使用/////////////////
    std::atomic_bool et_stop = false;

    //////////////////////////////////////以下为私有函数，由公有函数调用
    //基本参数设置
    void setBaseValue();



    //1.功能性函数，前进后退等
    //前进 function设为1
    void moveForward(int pulse);
    //后退 function设为2
    void moveBack(int pulse);
    //将当前位置设置为零点 function设为3
    void setZero();
    //回到原点 function设为4
    void backToZero();


    //选轴
    void decadeAxis(int axis);
    //轴置零
    void resetAxis();

    //单轴
    void singleAxis();

    void doubleAxis();

    void triAxis();

    //基础运动函数完成信号发送函数，线程
    void basedThreadSend();

    void speedCheck();//检查速度是否归零的函数


};
#endif