#include "MotorMove.h"

MotorMove::MotorMove(QObject* parent)
{

}
MotorMove::~MotorMove() {

}

//基础运动
void MotorMove::basedMove(int axis,int function,int pulse) {
	//判断出轴和轴数
	decadeAxis(axis);

	if (function == 1){
		moveForward(pulse);
	}
	if (function == 2) {
		moveBack(pulse);
	}
	if (function == 3) {
		setZero();
	}
	if (function == 4) {
		backToZero();
	}
	//执行完功能后，应将decadeAxis函数检测到的轴和轴数归零
	resetAxis();
}

/////////////////////////////////////基础运动//////////////////////////////
//前进
void MotorMove::moveForward(int pulse) {
	//只有一轴
	if (nums_of_axis == 1) {
		LC.nPulseNum = pulse;
		singleAxis();
	}
	if (nums_of_axis == 2) {
		LD.n1AxisPulseNum = pulse;
		LD.n2AxisPulseNum = pulse;
		doubleAxis();
	}
	if (nums_of_axis == 3) {

	}

}
//后退
void MotorMove::moveBack(int pulse) {
	//只有一轴
	if (nums_of_axis == 1) {
		LC.nPulseNum = pulse;
		singleAxis();
	}
	if (nums_of_axis == 2) {

	}
	if (nums_of_axis == 3) {

	}
}
//置零
void MotorMove::setZero() {

}
//归零
void MotorMove::backToZero() {

}
//////////////////////////////////////////////////////////////////
// /////////////////1轴，2轴，3轴///////////////////////////////
void MotorMove::singleAxis() {
	if (x_axis == 1) {
		LC.AxisNum = USB1020_XAXIS;
	}
	if (y_axis == 1) {
		LC.AxisNum = USB1020_YAXIS;
	}
	if (z_axis == 1) {
		LC.AxisNum = USB1020_ZAXIS;
	}
	LC.LV_DV = USB1020_DV;//定长运动
	LC.PulseMode = USB1020_CPDIR;//脉冲方式（另一种CWCCW）
	LC.Line_Curve = USB1020_LINE;//运动方式（直线加速度）
	DL.Multiple = 10;//倍率
	DL.Acceleration = 4000;//加速度和减速度
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;//初始速度
	DL.DriveSpeed = 8000;//最终速度

	USB1020_InitLVDV(hDevice, &DL, &LC);//将参数载入
	USB1020_StartLVDV(hDevice, LC.AxisNum);//开始运动

	basedThreadSend();//创建线程发送基础运动完成的函数
}
void MotorMove::doubleAxis() {
	if (x_axis * y_axis == 1) {//乘积为1，说明是这两个轴
		IA.Axis1 = USB1020_XAXIS;
		IA.Axis2 = USB1020_YAXIS;
	}
	if (x_axis * z_axis == 1) {
		IA.Axis1 = USB1020_XAXIS;
		IA.Axis2 = USB1020_ZAXIS;
	}
	if (y_axis * z_axis == 1) {
		IA.Axis1 = USB1020_YAXIS;
		IA.Axis2 = USB1020_ZAXIS;
	}
	//暂时不考虑脉冲输出的问题
	LD.Line_Curve = USB1020_LINE;//直线加速
	LD.ConstantSpeed = USB1020_CONSTAND;//固定速度

	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);//2D的参数载入
	USB1020_StartLineInterpolation_2D(hDevice);
}
void MotorMove::triAxis() {

}
//////////////////////////////////////////////////////////////////


//决定轴数;得出了有哪些轴需要运动和有几个轴
void MotorMove::decadeAxis(int axis) {
	if (axis & XAXIS) {
		x_axis = 1;
		++nums_of_axis;
	}
	if (axis & YAXIS) {
		y_axis = 1;
		++nums_of_axis;
	}
	if (axis & ZAXIS) {
		z_axis = 1;
		++nums_of_axis;
	}
}

//复位轴和咒术
void MotorMove::resetAxis() {
	x_axis = 0;
	y_axis = 0;
	z_axis = 0;
	nums_of_axis = 0;
}

///////////////////////////////线程函数///////////////////////////////////
//基础运动函数完成信号发送函数，线程
void MotorMove::basedThreadSend() {
	std::thread([&] {
		LONG speed1 = 0, speed2 = 0, speed3 = 0;//三个轴的速度
		unsigned int sign_of_stop = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed1 != 0 || speed2 != 0 || speed3 != 0) {//只要任意一个轴的速度不为0，就一直测
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		}
		if (x_axis == 1) {
			sign_of_stop = sign_of_stop | XAXIS;
		}
		if (y_axis == 1) {
			sign_of_stop = sign_of_stop | YAXIS;
		}
		if (z_axis == 1) {
			sign_of_stop = sign_of_stop | ZAXIS;
		}
		emit basedMoveComplate(sign_of_stop);
		}).detach();
}