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
		LD.n1AxisPulseNum = pulse;
		LD.n2AxisPulseNum = pulse;
		LD.n3AxisPulseNum = pulse;
		triAxis();
	}

}
//后退
void MotorMove::moveBack(int pulse) {
	//只有一轴
	if (nums_of_axis == 1) {
		LC.nPulseNum = -pulse;
		singleAxis();
	}
	if (nums_of_axis == 2) {
		LD.n1AxisPulseNum = -pulse;
		LD.n2AxisPulseNum = -pulse;
		doubleAxis();
	}
	if (nums_of_axis == 3) {
		LD.n1AxisPulseNum = -pulse;
		LD.n2AxisPulseNum = -pulse;
		LD.n3AxisPulseNum = -pulse;
		triAxis();
	}
}
//置零
void MotorMove::setZero() {
	if (x_axis == 1) {
		USB1020_SetLP(hDevice, USB1020_XAXIS, 0);
		USB1020_SetEP(hDevice, USB1020_XAXIS, 0);
	}
	if (y_axis == 1) {
		USB1020_SetLP(hDevice, USB1020_YAXIS, 0);
		USB1020_SetEP(hDevice, USB1020_YAXIS, 0);
	}
	if (z_axis == 1) {
		USB1020_SetLP(hDevice, USB1020_ZAXIS, 0);
		USB1020_SetEP(hDevice, USB1020_ZAXIS, 0);
	}
}
//归零
void MotorMove::backToZero() {
	LONG x_LP = USB1020_ReadLP(hDevice, USB1020_XAXIS);//读出三个轴的逻辑位置
	LONG y_LP = USB1020_ReadLP(hDevice, USB1020_YAXIS);
	LONG z_LP = USB1020_ReadLP(hDevice, USB1020_ZAXIS);

	if (x_axis == 1) {
		LC.LV_DV = USB1020_DV;
		LC.PulseMode = USB1020_CPDIR;
		LC.Line_Curve = USB1020_LINE;
		
		DL.Multiple = 10;
		DL.Acceleration = 4000;
		DL.Deceleration = 4000;
		DL.StartSpeed = 2000;
		DL.DriveSpeed = 8000;
		LC.AxisNum = USB1020_XAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(x_LP);
		if (x_LP > 0) {//脉冲大于0，反转归0；
			LC.Direction = USB1020_MDIRECTION;
		}else {
			LC.Direction = USB1020_PDIRECTION;
		}
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}
	if (y_axis == 1) {
		LC.LV_DV = USB1020_DV;
		LC.PulseMode = USB1020_CPDIR;
		LC.Line_Curve = USB1020_LINE;

		DL.Multiple = 10;
		DL.Acceleration = 4000;
		DL.Deceleration = 4000;
		DL.StartSpeed = 2000;
		DL.DriveSpeed = 8000;
		LC.AxisNum = USB1020_YAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(y_LP);
		if (y_LP > 0) {//脉冲大于0，反转归0；
			LC.Direction = USB1020_MDIRECTION;
		}
		else {
			LC.Direction = USB1020_PDIRECTION;
		}
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}
	if (z_axis == 1) {
		LC.LV_DV = USB1020_DV;
		LC.PulseMode = USB1020_CPDIR;
		LC.Line_Curve = USB1020_LINE;

		DL.Multiple = 10;
		DL.Acceleration = 4000;
		DL.Deceleration = 4000;
		DL.StartSpeed = 2000;
		DL.DriveSpeed = 8000;
		LC.AxisNum = USB1020_ZAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(z_LP);
		if (z_LP > 0) {//脉冲大于0，反转归0；
			LC.Direction = USB1020_MDIRECTION;
		}
		else {
			LC.Direction = USB1020_PDIRECTION;
		}
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}

	basedThreadSend();
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

	basedThreadSend();//创建线程发送基础运动完成的函数
}
void MotorMove::triAxis() {
	IA.Axis1 = USB1020_XAXIS;
	IA.Axis2 = USB1020_YAXIS;
	IA.Axis3 = USB1020_ZAXIS;

	//暂时不考虑脉冲输出模式
	LD.Line_Curve = USB1020_LINE;//直线运动
	LD.ConstantSpeed = USB1020_CONSTAND;//都设置为固定速度

	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	USB1020_InitLineInterpolation_3D(hDevice, &DL, &IA, &LD);
	USB1020_StartLineInterpolation_3D(hDevice);

	basedThreadSend();//创建线程发送基础运动完成的函数
}
//////////////////////////////////////////////////////////////////

////////////////////////////////选轴和轴置零/////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////
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
////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////z轴的循环运动和相关函数////////////////////////////////////////////////////////////

void MotorMove::zAxisLoopMove(int steps, int step_distance) {
	LC.AxisNum = USB1020_ZAXIS;
	int output = 1;
	int n = (steps - 1) / 2;//得到每一边有多少点
	{
		LC.Direction = 1;
		LC.nPulseNum = n * step_distance;//先走到最右边
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		
		zAxisThreadSend(output);
	}
	for (int i = 0; i < 2 * n; ++i) {
		++output;
		LC.Direction = 0;
		LC.nPulseNum = step_distance;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		zAxisThreadSend(output);
	}
	{
		LC.Direction = 1;
		LC.nPulseNum = n * step_distance;//走回中点
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}
}

//z轴的信号发送和线程处理
void MotorMove::zAxisThreadSend(int output) {
	std::thread([&] {
		LONG speed = 0;
		speed = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		}
		emit zAxisLoopMoveComplate(output);//发送这是第几次

		}).detach();
}
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////xy的“弓”型运动/////////////////////////////////////////////
void MotorMove::xyAxisMove(int length, int width, int level_distance, int vertical_distance) {
	int location_x = 0, location_y = 0;//xy的坐标点
	//外部循环为y，内部循环为x
	for (int j = 0; j < width; ++j) {
		LC.AxisNum = USB1020_XAXIS;
		if (j % 2 == 0)LC.Direction = 1;
		else LC.Direction = 0;
		//先让x运动，在让y运动
		for (int i = 0; i < length; ++i) {
			LC.nPulseNum = level_distance;
			USB1020_InitLVDV(hDevice, &DL, &LC);
			USB1020_StartLVDV(hDevice, LC.AxisNum);
			if (LC.Direction == 1) { xyAxisThreadSend(i + 1, j + 1); }
			else { xyAxisThreadSend(length - i, j + 1); }
		}
		LC.AxisNum = USB1020_YAXIS;
		LC.Direction = 1;//y轴始终往下走
		LC.nPulseNum = vertical_distance;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		if (j % 2 == 0) { xyAxisThreadSend(length, j + 1); }//偶数时，位置在每行的末尾
		else { xyAxisThreadSend(1, j + 1); }//奇数时，在每行的开头
	}
}

//xy轴运动的信号发送
void MotorMove::xyAxisThreadSend(int location_x, int location_y) {
	std::thread([&] {
		LONG speed1 = 0,speed2=0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);

		while (speed1!= 0||speed2!=0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		}
		emit xyAxisMoveComplate(location_x, location_y);
		}).detach();
}