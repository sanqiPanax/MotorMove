#ifndef BUILD_STATIC
#define BUILD_STATIC
#endif // !BUILD_STATIC
#include "MotorMove.h"

MotorMove::MotorMove(QObject* parent)
{
	try {
		hDevice = USB1020_CreateDevice(0);
	}
	catch (const std::string& s) {
		QMessageBox::warning(NULL, "warning", QString::fromLocal8Bit("运动控制设备初始化失败"), QMessageBox::Yes, QMessageBox::Yes);
	}

	//设置硬件限位停止有效
	USB1020_SetMDirLMTEnable(hDevice, USB1020_XAXIS, USB1020_SUDDENSTOP, 0);
	USB1020_SetMDirLMTEnable(hDevice, USB1020_YAXIS, USB1020_SUDDENSTOP, 0);
	USB1020_SetMDirLMTEnable(hDevice, USB1020_ZAXIS, USB1020_SUDDENSTOP, 0);

	LC.Direction = 0;

	readNowLocation();//启动循环读取位置的线程
}
MotorMove::~MotorMove() {
	pos_surveil = false;
	for (; pos_destroyed != true;) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	USB1020_ReleaseDevice(hDevice);
}

//基础运动
void MotorMove::basedMove(int axis, int function, int pulse) {
	//判断出轴和轴数
	decadeAxis(axis);

	if (function == 1) {
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

/////////////////////////////////////基础运动/////////////////////////////////////////////
//前进
void MotorMove::moveForward(int pulse) {
	//只有一轴
	if (nums_of_axis == 1) {
		LC.nPulseNum = pulse;
		LC.Direction = 1;
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
		LC.nPulseNum = pulse;
		LC.Direction = 0;
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
		USB1020_SetLP(hDevice, USB1020_XAXIS, 0);//将逻辑位置和实际位置都设为0
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

		DL.Multiple = 15;
		DL.Acceleration = 2000;
		DL.Deceleration = 2000;
		DL.StartSpeed = 1000;
		DL.DriveSpeed = 3000;
		LC.AxisNum = USB1020_XAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(x_LP);
		//if (x_LP > 0) {//脉冲大于0，反转归0；
		//	LC.Direction = USB1020_MDIRECTION;
		//}else {
		//	LC.Direction = USB1020_PDIRECTION;
		//}
		LC.Direction = x_LP > 0 ? 0 : 1;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}
	if (y_axis == 1) {
		LC.LV_DV = USB1020_DV;
		LC.PulseMode = USB1020_CPDIR;
		LC.Line_Curve = USB1020_LINE;

		DL.Multiple = 15;
		DL.Acceleration = 2000;
		DL.Deceleration = 2000;
		DL.StartSpeed = 1000;
		DL.DriveSpeed = 3000;
		LC.AxisNum = USB1020_YAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(y_LP);
		//if (y_LP > 0) {//脉冲大于0，反转归0；
		//	LC.Direction = USB1020_MDIRECTION;
		//}
		//else {
		//	LC.Direction = USB1020_PDIRECTION;
		//}
		LC.Direction = y_LP > 0 ? 0 : 1;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
	}
	if (z_axis == 1) {
		LC.LV_DV = USB1020_DV;
		LC.PulseMode = USB1020_CPDIR;
		LC.Line_Curve = USB1020_LINE;

		DL.Multiple = 15;
		DL.Acceleration = 2000;
		DL.Deceleration = 2000;
		DL.StartSpeed = 1000;
		DL.DriveSpeed = 3000;
		LC.AxisNum = USB1020_ZAXIS;
		//先在前进后退中测试能不能仅靠改变脉冲的正负，改变移动方向，如果可以就在这里将方向去掉
		LC.nPulseNum = abs(z_LP);
		//if (z_LP > 0) {//脉冲大于0，反转归0；
		//	LC.Direction = USB1020_MDIRECTION;
		//}
		//else {
		//	LC.Direction = USB1020_PDIRECTION;
		//}
		LC.Direction = z_LP > 0 ? 0 : 1;
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
	LC.DecMode = USB1020_AUTO;
	DL.Multiple = 15;//倍率
	DL.Acceleration = 2000;//加速度和减速度
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;//初始速度
	DL.DriveSpeed = 3000;//最终速度

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

	LD.Line_Curve = USB1020_LINE;//直线加速
	LD.ConstantSpeed = USB1020_CONSTAND;//固定速度

	DL.Multiple = 15;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 3000;

	USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);//2D的参数载入
	USB1020_StartLineInterpolation_2D(hDevice);

	basedThreadSend();//创建线程发送基础运动完成的函数
}
void MotorMove::triAxis() {
	IA.Axis1 = USB1020_XAXIS;
	IA.Axis2 = USB1020_YAXIS;
	IA.Axis3 = USB1020_ZAXIS;

	LD.Line_Curve = USB1020_LINE;//直线运动
	LD.ConstantSpeed = USB1020_CONSTAND;//都设置为固定速度

	DL.Multiple = 15;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 3000;

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
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed1 != 0 || speed2 != 0 || speed3 != 0) {//只要任意一个轴的速度不为0，就一直测
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		//好像没有必要，将原本传入的全部传回即可
		emit basedMoveComplate();
		}).detach();
}
/////////////////////////////z轴的循环运动和相关函数///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////开头调用一次zAxisLoopMove，然后开始发送信号给槽函数，接收信号，
/////////////////////最后走完所有点之后，再调用一次zAxisLoopMove

void MotorMove::zAxisLoopMove(int steps, int step_distance) {
	zAxis_pulse = step_distance;//给全局变量步长赋值
	zAxis_steps = steps;
	zAxis_half_steps = (steps - 1) / 2;
	//定长驱动
	LC.LV_DV = USB1020_DV;
	//脉冲方式
	LC.PulseMode = USB1020_CPDIR;
	//运动方式（直线加减速还是曲线加减速）
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 15;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 3000;

	LC.AxisNum = USB1020_ZAXIS;
	LC.nPulseNum = zAxis_pulse * zAxis_half_steps;
	LC.Direction = 0;

	//zAxisThreadSend(1);
}
//z轴的信号发送和线程处理
void MotorMove::zAxisThreadSend() {
	if (zAxis_steps < 0) {
		return;
	}
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed = 0;
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}

		--zAxis_steps;//步长减一
		if (zAxis_steps == 0) {//切换方向，下一次回原点
			LC.Direction = 0;
			LC.nPulseNum = zAxis_pulse * zAxis_half_steps;
			emit zAxisLoopMoveComplate();
		}
		else {
			LC.Direction = 1;
			LC.nPulseNum = zAxis_pulse;
			emit zAxisLoopMoveComplate();
		}
		}).detach();
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////xy的“弓”型运动///////////////////////////////////////////

void MotorMove::xyAxisMove(int length, int width, double level_distance, double vertical_distance) {
	xLength = length;
	yWidth = width;
	xDistance = level_distance;
	yDistance = vertical_distance;
	//定长驱动
	LC.LV_DV = USB1020_DV;
	//脉冲方式
	LC.PulseMode = USB1020_CPDIR;
	//运动方式（直线加减速还是曲线加减速）
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 15;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 3000;

	LC.AxisNum = USB1020_XAXIS;
	LC.Direction = 1;
	LC.nPulseNum = xDistance;

	yCount = 0;//将上次的清零

}
///先执行一次xyAxisMove，然后进入处理图像，发送信号的循环
//xy轴运动的信号发送
void MotorMove::xyAxisThreadSend() {
	++xyTimes;
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed1 = 0, speed2 = 0;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		
		if (xyTimes == xLength) {
			LC.AxisNum = USB1020_YAXIS;
			LC.nPulseNum = yDistance;
			LC.Direction = 1;
			++yCount;
		}
		if (xyTimes > xLength) {
			xyTimes = 1;
			LC.AxisNum = USB1020_XAXIS;
			LC.nPulseNum = xDistance;
			if (yCount % 2 == 1) {
				LC.Direction = 0;
			}
			else
			{
				LC.Direction = 1;
			}
		}
		emit xyAxisMoveComplate();
		}).detach();
}


//改变全局变量clear_value的值
void MotorMove::getClearValue(double value) {
	clear_value = value;
}
void MotorMove::etStop()
{
	et_stop = true;
}
//移动到最清晰的位置
void MotorMove::moveToMostClearPoint(double et_threshold, int pulseDistance) {
	if (pulseDistance > 5000) return;
	//先设置运动参数
	LC.LV_DV = USB1020_DV;
	LC.PulseMode = USB1020_CPDIR;
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 3;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 3000;
	LC.AxisNum = USB1020_ZAXIS;

	/// <summary>
	/// /上面不需要管/////////////////////////////////////////
	/// </summary>
	/// <param name="pulseDistance"></param>

	et_stop = false;
	std::atomic_bool direct_right = 0;
	emit updateValue();//获得清晰值

	LC.nPulseNum = pulseDistance;//初始步长
	LC.Direction = USB1020_PDIRECTION;//初始方向默认为正
	int pre_direction = LC.Direction;
	int move_times = 0,check_times = 0;
	double pre_value = 0, begin_value = clear_value;
	while (clear_value < et_threshold && clear_value>10&&move_times <100) {//
		pre_value = clear_value;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		speedCheck();//调用速度检查函数，速度为0后，会继续执行
		emit updateValue();//更新清晰度
		while ((pre_value - clear_value) == 0) {
			emit updateValue();
			++check_times;
			if (check_times > 50) break;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (abs((pre_value - clear_value) < 2)) {
			LC.nPulseNum = pulseDistance * 4;
		}
		if (clear_value == 0) {//情况1：可能跑出界的情况 此时先回到原位置，然后步长减半，继续走
			LC.Direction = (pre_direction == USB1020_MDIRECTION) ? USB1020_PDIRECTION : USB1020_MDIRECTION;//方向反向
			pre_direction = LC.Direction;
			USB1020_InitLVDV(hDevice, &DL, &LC);
			USB1020_StartLVDV(hDevice, LC.AxisNum);
			speedCheck();//已回到原位置
			LC.Direction = (pre_direction == USB1020_MDIRECTION) ? USB1020_PDIRECTION : USB1020_MDIRECTION;//方向反向
			pre_direction = LC.Direction;

			LC.nPulseNum /= 2;//步长减半
		}
		else if ((begin_value - clear_value) > 2) {//清晰度反而减小了，说明可能走反了,只需反个向就好
			LC.nPulseNum = pulseDistance;
			LC.Direction = USB1020_MDIRECTION;//方向反向
			pre_direction = LC.Direction;
			if (direct_right == true) break;
		}
		else if ((begin_value - clear_value) < -2) {//清晰度变大了，说明可能走对了
			direct_right = true;
			//LC.nPulseNum = pulseDistance;
			//LC.nPulseNum -= 2;
		}
		//if (abs(clear_value - et_threshold) < 3) LC.nPulseNum = pulseDistance / 2;
		if (LC.nPulseNum >= pulseDistance * 150) break;
		if (et_stop == true) break;
		++move_times;
	}
	//循环结束加个急停，以防撞车
	USB1020_InstStop(hDevice, LC.AxisNum);
}
/////////////////////////////////槽函数/////////////////////////离焦曲线
void MotorMove::bounryMove(double journey) {

	//if (now_location<lower_bounry || now_location>higer_bounry)return now_location;
	LC.LV_DV = USB1020_DV;
	LC.PulseMode = USB1020_CPDIR;
	LC.Line_Curve = USB1020_LINE;

	DL.Multiple = 3;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 6000;
	LC.AxisNum = USB1020_ZAXIS;
	std::thread([&] {
		LC.Direction = 1;
		LC.nPulseNum = journey / 0.0125 / 2;//脉冲大小为两者之差
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		long speed = USB1020_ReadCV(hDevice, LC.AxisNum);//读取当前速度
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//不停读取速度知道停下来
		}
		emit arrivedBounryBegin();//到达其中一个边界

		LC.Direction = 0;
		LC.nPulseNum = journey / 0.0125;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//不停读取速度知道停下来
		}
		emit arrivedBounryEnd();//到达另一个边界

		//回到之前的点
		LC.Direction = 1;
		LC.nPulseNum = journey / 0.0125 / 2;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//不停读取速度知道停下来
		}
		}).detach();

}
//走到xy轴最极限位置,然后在走回来
void MotorMove::moveToBorder(double x_axis_back, double y_axis_back) {
	std::thread([&] {
		//USB1020_SetStopEnable(hDevice, USB1020_XAXIS, USB1020_IN0, 0);
		DL.Multiple = 15;
		DL.Acceleration = 2000;
		DL.Deceleration = 2000;
		DL.StartSpeed = 1000;
		DL.DriveSpeed = 3000;

		IA.Axis1 = USB1020_XAXIS;
		IA.Axis2 = USB1020_YAXIS;
		IA.Axis3 = USB1020_ZAXIS;

		LD.Line_Curve = USB1020_LINE;//直线运动
		LD.ConstantSpeed = USB1020_CONSTAND;//都设置为固定速度


		LD.n1AxisPulseNum = 200000;
		LD.n2AxisPulseNum = 200000;
		LD.n3AxisPulseNum = 0;
		USB1020_InitLineInterpolation_3D(hDevice, &DL, &IA, &LD);
		USB1020_StartBitInterpolation_3D(hDevice);

		LONG speed1 = 0, speed2 = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);

		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		//这里应该将位置设置好，之后走到位置后也将位置设置好
		//走到极限后根据参数往回走
		LD.n1AxisPulseNum = x_axis_back;
		LD.n2AxisPulseNum = y_axis_back;
		USB1020_InitLineInterpolation_3D(hDevice, &DL, &IA, &LD);
		USB1020_StartBitInterpolation_3D(hDevice);

		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		}).detach();
}

//需要z轴运行范围
void MotorMove::locationSet(double length) {
	LC.LV_DV = USB1020_DV;
	LC.PulseMode = USB1020_CPDIR;
	LC.Line_Curve = USB1020_LINE;

	DL.Multiple = 3;
	DL.Acceleration = 2000;
	DL.Deceleration = 2000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 6000;
	LC.AxisNum = USB1020_ZAXIS;
	LC.Direction = 1;
	LC.nPulseNum = 200000;//设置20万的脉冲，直接走到硬件极限位置
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		long speed = USB1020_ReadCV(hDevice, LC.AxisNum);//读取当前速度
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//不停读取速度知道停下来
		}
		LC.nPulseNum = length;//数据库中的z轴长度
		LC.Direction = 0;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);//读取当前速度
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//不停读取速度知道停下来
		}
		}).detach();

}

//不停地读取xyz当前的位置，每隔一段时间发送一次信号
void MotorMove::readNowLocation() {
	pos_surveil = true;
	std::thread([&] {
		while (true)
		{
			if (pos_surveil != true) break;
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			emit showNowLocation(USB1020_ReadLP(hDevice,USB1020_XAXIS),USB1020_ReadLP(hDevice,USB1020_YAXIS),USB1020_ReadEP(hDevice,USB1020_ZAXIS));
		}
		pos_destroyed = true;
		}).detach();
}

/////////////////////////////私有函数//////////////////////////////////////

void MotorMove::speedCheck() {
	LONG speed = USB1020_ReadCV(hDevice, LC.Direction);
	while (1) {
		speed = USB1020_ReadCV(hDevice, LC.Direction);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		if (speed == 0)break;
	}
}
