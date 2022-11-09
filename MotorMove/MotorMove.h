#ifndef _MOTOR_MOVE_H_
#define _MOTOR_MOVE_H_

#include "motormove_global.h"
#include <iostream>
#include "USB1020.h"
#include <thread>

//const int XAXIS = 0b00000001;
//const int YAXIS = 0b00000010;
//const int ZAXIS = 0b00000100;
//
//const int XYAXIS = 0b00000011;
//const int XZAXIS = 0b00000101;
//const int YZAXIS = 0b00000110;
//const int XYZAXIS = 0b00000111;
//
//const int FORWARD = 1;
//const int BACK = 2;
//const int SETZERO = 3;
//const int BACKZERO = 4;

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

    MotorMove(QObject* parent=Q_NULLPTR);
    ~MotorMove();


    // (�βΣ���Щ�ᣨ�����Ʊ�ʾ����ִ�й��ܣ�������)�����˶����������Ļ����˶����ܣ������������ǰ�����ˣ����㣬��λ
    void basedMove(int axis,int function,int pulse);
    // ���βΣ�������4����3��λ�ã�8����5��λ�ã�12����7��λ��),���ࣨÿ���೤������
    void zAxisLoopMove(int steps,int step_distance);
    // ���βΣ�����ĳ���ÿ�ж��ٿף�ÿ�ж��ٸ��ף���ˮƽ�׾ࣨ�������ڿ�֮��ľ��룩����ֱ�׾ࣩ xy�岹�˶���
    void xyAxisMove(int length,int width,int level_distance,int vertical_distance);
    //z����źŷ��ͺ��̴߳���
    void zAxisThreadSend();
    //xy����źŷ��ͺ��̴߳���
    //xy�ᡰ�������˶����źŷ��ͺ��̴߳���
    void xyAxisThreadSend();

signals:
    void basedMoveComplate();//�����˶�����źţ����������������
    void zAxisLoopMoveComplate();//�βξ��ǵڶ��ٴΣ�����˵�ڶ��ٲ���
    void xyAxisMoveComplate();//�βξ��ǵ�ǰλ��


private:
    //�豸�����������ṹ���
    HANDLE hDevice;
    
    USB1020_PARA_DataList DL;
    
    USB1020_PARA_LCData LC;
    
    USB1020_PARA_LineData LD;//�����������Ҫ�ı���

    USB1020_PARA_InterpolationAxis IA;

    /////////////////////////�Զ����˽�б���////////////////////////////////
    int x_axis = 0;
    int y_axis = 0;
    int z_axis = 0;
    int nums_of_axis = 0;

    ////////////////z��ʹ��////////////////////////////
    int zAxisPulse = 0;
    int zAxisSteps = 0;
    ////////////////xy��ʹ��///////////////////////////
    int xLength = 0;
    int yWidth = 0;
    int xDistance = 0;
    int yDistance = 0;
    int xyTimes = 1;
    int yCount = 0;//�����л�����

    /////////////////////////////////////////////////////////

    //����Ϊ˽�к������ɹ��к�������

    //1.�����Ժ�����ǰ�����˵�
    //ǰ�� function��Ϊ1
    void moveForward(int pulse);
    //���� function��Ϊ2
    void moveBack(int pulse);
    //����ǰλ������Ϊ��� function��Ϊ3
    void setZero();
    //�ص�ԭ�� function��Ϊ4
    void backToZero();

    
    //ѡ��
    void decadeAxis(int axis);
    //������
    void resetAxis();

    //����
    void singleAxis();

    void doubleAxis();

    void triAxis();

    //3.�����Ϣ
    //�����ǰλ��
    void showCurrentLocation();

    //�����˶���������źŷ��ͺ������߳�
    void basedThreadSend();

 
};
#endif