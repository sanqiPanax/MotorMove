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


    // (�βΣ���Щ�ᣨ�����Ʊ�ʾ����ִ�й��ܣ�������)�����˶����������Ļ����˶����ܣ������������ǰ�����ˣ����㣬��λ
    void basedMove(int axis, int function, int pulse);
    // ���βΣ�������4����3��λ�ã�8����5��λ�ã�12����7��λ��),���ࣨÿ���೤������
    void zAxisLoopMove(int steps, int step_distance);
    // ���βΣ�����ĳ���ÿ�ж��ٿף�ÿ�ж��ٸ��ף���ˮƽ�׾ࣨ�������ڿ�֮��ľ��룩����ֱ�׾ࣩ xy�岹�˶���
    void xyAxisMove(int length, int width, double level_distance, double vertical_distance);
    //z����źŷ��ͺ��̴߳���
    void zAxisThreadSend();
    //xy����źŷ��ͺ��̴߳���
    //xy�ᡰ�������˶����źŷ��ͺ��̴߳���
    void xyAxisThreadSend();
    //�����ǰλ��
    void showCurrentLocation();

    //�ƶ�z�ᣬ�ߵ��������ĵ�
    //void moveToMostClearPoint(int large_range,int small_range);
    void moveToMostClearPoint(double et_threshold, int pulseDistance);

    //��xy�ƶ����߽磬�͡��������˶��ķ����෴ 
    void moveToBorder(double x_axis_back,double y_axis_back);

signals:
    void basedMoveComplate();//�����˶�����źţ����������������
    void zAxisLoopMoveComplate();//�βξ��ǵڶ��ٴΣ�����˵�ڶ��ٲ���
    void xyAxisMoveComplate();//�βξ��ǵ�ǰλ��
    void startMeasure();
    void createError();
    void getMidCameraMtf(double& inp_mtf);

    void arrivedBounryBegin();//����߽緢�͵��ź�
    void arrivedBounryEnd();

    void updateValue();
    void changeChearPointValue(double mtf_value);
    void showNowLocation(double xAxisLocation,double yAxisLocation,double zAxisLocation);//���͵�ǰλ�õ��ź�

    
public slots:
    void bounryMove(double journey);//�������߽�֮���ƶ�,Ҫ�ڻ�ȡ�˵�ǰλ�õ���

    void getClearValue(double value);//��õ�ǰ�����ȵĺ������ͶԷ����ź�����

    void etStop();//�ж��Զ�����
private:
    //�豸�����������ṹ���
    HANDLE hDevice;

    USB1020_PARA_DataList DL;

    USB1020_PARA_LCData LC;

    USB1020_PARA_LineData LD;//�����������Ҫ�ı���

    USB1020_PARA_InterpolationAxis IA;

    //��ָ����������
    USB1020_PARA_DataList* DL1 = new USB1020_PARA_DataList();
    USB1020_PARA_LCData* LC1 = new USB1020_PARA_LCData();
    USB1020_PARA_LineData* LD1 = new USB1020_PARA_LineData();
    ////////////////////�Զ����˽�б���////////////////
    int x_axis = 0;
    int y_axis = 0;
    int z_axis = 0;
    int nums_of_axis = 0;

    ////////////////z��ʹ��////////////////////////////
    int zAxis_pulse = 0;
    int zAxis_steps = 0;//һ��
    int zAxis_half_steps = 0;//һ��
    ////////////////xy��ʹ��///////////////////////////
    int xLength = 0;
    int yWidth = 0;
    int xDistance = 0;
    int yDistance = 0;
    int xyTimes = 1;
    int yCount = 0;//�����л�����
    /////////////////////////////////////////////////////////
    double clear_value = 0;
    double now_location = 0;//���ڵ�λ��
    ///////////////�Զ�����ʹ��/////////////////
    std::atomic_bool et_stop = false;

    //////////////////////////////////////����Ϊ˽�к������ɹ��к�������
    //������������
    void setBaseValue();



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

    //�����˶���������źŷ��ͺ������߳�
    void basedThreadSend();

    void speedCheck();//����ٶ��Ƿ����ĺ���


};
#endif