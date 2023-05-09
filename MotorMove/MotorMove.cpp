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
		std::cerr << "Error: " << "�豸��ʼ��ʧ��" << std::endl;
	}
	//�������������ʽ
	USB1020_PulseOutMode(hDevice, USB1020_XAXIS, USB1020_CPDIR, 0, 0);
	USB1020_PulseOutMode(hDevice, USB1020_YAXIS, USB1020_CPDIR, 0, 0);
	USB1020_PulseOutMode(hDevice, USB1020_ZAXIS, USB1020_CPDIR, 0, 0);

	//����Ӳ����λֹͣ��Ч
	USB1020_SetMDirLMTEnable(hDevice, USB1020_XAXIS, USB1020_SUDDENSTOP, 0);
	USB1020_SetMDirLMTEnable(hDevice, USB1020_YAXIS, USB1020_SUDDENSTOP, 0);
	USB1020_SetMDirLMTEnable(hDevice, USB1020_ZAXIS, USB1020_SUDDENSTOP, 0);

	setBaseValue();//������������
}
MotorMove::~MotorMove() {
	USB1020_ReleaseDevice(hDevice);
}

//�����˶�
void MotorMove::basedMove(int axis, int function, int pulse) {
	//�жϳ��������
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
	//ִ���깦�ܺ�Ӧ��decadeAxis������⵽�������������
	resetAxis();
}

/////////////////////////////////////�����˶�//////////////////////////////
//ǰ��
void MotorMove::moveForward(int pulse) {
	//ֻ��һ��
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
//����
void MotorMove::moveBack(int pulse) {
	//ֻ��һ��
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
//����
void MotorMove::setZero() {
	if (x_axis == 1) {
		USB1020_SetLP(hDevice, USB1020_XAXIS, 0);//���߼�λ�ú�ʵ��λ�ö���Ϊ0
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
//����
void MotorMove::backToZero() {
	LONG x_LP = USB1020_ReadLP(hDevice, USB1020_XAXIS);//������������߼�λ��
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
		//����ǰ�������в����ܲ��ܽ����ı�������������ı��ƶ�����������Ծ������ｫ����ȥ��
		LC.nPulseNum = abs(x_LP);
		//if (x_LP > 0) {//�������0����ת��0��
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

		DL.Multiple = 10;
		DL.Acceleration = 4000;
		DL.Deceleration = 4000;
		DL.StartSpeed = 2000;
		DL.DriveSpeed = 8000;
		LC.AxisNum = USB1020_YAXIS;
		//����ǰ�������в����ܲ��ܽ����ı�������������ı��ƶ�����������Ծ������ｫ����ȥ��
		LC.nPulseNum = abs(y_LP);
		//if (y_LP > 0) {//�������0����ת��0��
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

		DL.Multiple = 10;
		DL.Acceleration = 4000;
		DL.Deceleration = 4000;
		DL.StartSpeed = 2000;
		DL.DriveSpeed = 8000;
		LC.AxisNum = USB1020_ZAXIS;
		//����ǰ�������в����ܲ��ܽ����ı�������������ı��ƶ�����������Ծ������ｫ����ȥ��
		LC.nPulseNum = abs(z_LP);
		//if (z_LP > 0) {//�������0����ת��0��
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
// /////////////////1�ᣬ2�ᣬ3��///////////////////////////////
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
	LC.LV_DV = USB1020_DV;//�����˶�
	LC.PulseMode = USB1020_CPDIR;//���巽ʽ����һ��CWCCW��
	LC.Line_Curve = USB1020_LINE;//�˶���ʽ��ֱ�߼��ٶȣ�
	DL.Multiple = 10;//����
	DL.Acceleration = 4000;//���ٶȺͼ��ٶ�
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;//��ʼ�ٶ�
	DL.DriveSpeed = 8000;//�����ٶ�

	USB1020_InitLVDV(hDevice, &DL, &LC);//����������
	USB1020_StartLVDV(hDevice, LC.AxisNum);//��ʼ�˶�

	basedThreadSend();//�����̷߳��ͻ����˶���ɵĺ���
}
void MotorMove::doubleAxis() {
	if (x_axis * y_axis == 1) {//�˻�Ϊ1��˵������������
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

	LD.Line_Curve = USB1020_LINE;//ֱ�߼���
	LD.ConstantSpeed = USB1020_CONSTAND;//�̶��ٶ�

	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);//2D�Ĳ�������
	USB1020_StartLineInterpolation_2D(hDevice);

	basedThreadSend();//�����̷߳��ͻ����˶���ɵĺ���
}
void MotorMove::triAxis() {
	IA.Axis1 = USB1020_XAXIS;
	IA.Axis2 = USB1020_YAXIS;
	IA.Axis3 = USB1020_ZAXIS;

	LD.Line_Curve = USB1020_LINE;//ֱ���˶�
	LD.ConstantSpeed = USB1020_CONSTAND;//������Ϊ�̶��ٶ�

	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	USB1020_InitLineInterpolation_3D(hDevice, &DL, &IA, &LD);
	USB1020_StartLineInterpolation_3D(hDevice);

	basedThreadSend();//�����̷߳��ͻ����˶���ɵĺ���
}
//////////////////////////////////////////////////////////////////

////////////////////////////////ѡ���������/////////////////////////
//��������;�ó�������Щ����Ҫ�˶����м�����
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

//��λ�������
void MotorMove::resetAxis() {
	x_axis = 0;
	y_axis = 0;
	z_axis = 0;
	nums_of_axis = 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////�̺߳���///////////////////////////////////
//�����˶���������źŷ��ͺ������߳�
void MotorMove::basedThreadSend() {
	std::thread([&] {
		LONG speed1 = 0, speed2 = 0, speed3 = 0;//��������ٶ�
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed1 != 0 || speed2 != 0 || speed3 != 0) {//ֻҪ����һ������ٶȲ�Ϊ0����һֱ��
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		//����û�б�Ҫ����ԭ�������ȫ�����ؼ���
		emit basedMoveComplate();
		emit showNowLocation(USB1020_ReadLP(hDevice, USB1020_XAXIS), USB1020_ReadLP(hDevice, USB1020_YAXIS), USB1020_ReadEP(hDevice, USB1020_ZAXIS));
		}).detach();
}
/////////////////////////////z���ѭ���˶�����غ���///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////��ͷ����һ��zAxisLoopMove��Ȼ��ʼ�����źŸ��ۺ����������źţ�
/////////////////////����������е�֮���ٵ���һ��zAxisLoopMove

void MotorMove::zAxisLoopMove(int steps, int step_distance) {
	zAxis_pulse = step_distance;//��ȫ�ֱ���������ֵ
	zAxis_steps = steps;
	zAxis_half_steps = (steps - 1) / 2;
	//��������
	LC.LV_DV = USB1020_DV;
	//���巽ʽ
	LC.PulseMode = USB1020_CPDIR;
	//�˶���ʽ��ֱ�߼Ӽ��ٻ������߼Ӽ��٣�
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	LC.AxisNum = USB1020_ZAXIS;
	LC.nPulseNum = zAxis_pulse * zAxis_half_steps;
	LC.Direction = 0;

	//zAxisThreadSend(1);
}
//z����źŷ��ͺ��̴߳���
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
		emit zAxisLoopMoveComplate();
		emit showNowLocation(USB1020_ReadLP(hDevice, USB1020_XAXIS), USB1020_ReadLP(hDevice, USB1020_YAXIS), USB1020_ReadEP(hDevice, USB1020_ZAXIS));
		LC.Direction = 1;
		LC.nPulseNum = zAxis_pulse;

		--zAxis_steps;//������һ
		if (zAxis_steps == 0) {//�л�������һ�λ�ԭ��
			LC.Direction = 0;
			LC.nPulseNum = zAxis_pulse * zAxis_half_steps;
		}
		}).detach();
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////xy�ġ��������˶�///////////////////////////////////////////

void MotorMove::xyAxisMove(int length, int width, double level_distance, double vertical_distance) {
	xLength = length;
	yWidth = width;
	xDistance = level_distance;
	yDistance = vertical_distance;
	//��������
	LC.LV_DV = USB1020_DV;
	//���巽ʽ
	LC.PulseMode = USB1020_CPDIR;
	//�˶���ʽ��ֱ�߼Ӽ��ٻ������߼Ӽ��٣�
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	LC.AxisNum = USB1020_XAXIS;
	LC.Direction = 1;
	LC.nPulseNum = xDistance;

	yCount = 0;//���ϴε�����

}
///��ִ��һ��xyAxisMove��Ȼ����봦��ͼ�񣬷����źŵ�ѭ��
//xy���˶����źŷ���
void MotorMove::xyAxisThreadSend() {
	++xyTimes;
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed1 = 0, speed2 = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);

		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		emit xyAxisMoveComplate();
		emit showNowLocation(USB1020_ReadLP(hDevice, USB1020_XAXIS), USB1020_ReadLP(hDevice, USB1020_YAXIS), USB1020_ReadEP(hDevice, USB1020_ZAXIS));
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
		}).detach();
}



//�����ǰ�߼�λ�ú�ʵ��λ�ã�����еĻ�
void MotorMove::showCurrentLocation() {

	qDebug() << "xAxisLogicLocation��" << USB1020_ReadLP(hDevice, USB1020_XAXIS) << endl;
	qDebug() << "xAxisRealLocation��" << USB1020_ReadEP(hDevice, USB1020_XAXIS) << endl;

	qDebug() << "yAxisLogicLocation��" << USB1020_ReadLP(hDevice, USB1020_YAXIS) << endl;
	qDebug() << "yAxisRealLocation��" << USB1020_ReadEP(hDevice, USB1020_YAXIS) << endl;

	qDebug() << "zAxisLogicLocation����" << USB1020_ReadLP(hDevice, USB1020_ZAXIS) << endl;
	qDebug() << "zAxisRealLocation��" << USB1020_ReadEP(hDevice, USB1020_ZAXIS) << endl;
	
}
//�ı�ȫ�ֱ���clear_value��ֵ
void MotorMove::getClearValue(double value) {
	clear_value = value;
}
void MotorMove::etStop()
{
	et_stop = true;
}
//�ƶ�����������λ��
void MotorMove::moveToMostClearPoint(double et_threshold, int pulseDistance) {
	if (pulseDistance > 5000) return;
	//�������˶�����
	LC.LV_DV = USB1020_DV;
	LC.PulseMode = USB1020_CPDIR;
	LC.Line_Curve = USB1020_LINE;
	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;
	LC.AxisNum = USB1020_ZAXIS;

	/// <summary>
	/// /���治��Ҫ��/////////////////////////////////////////
	/// </summary>
	/// <param name="pulseDistance"></param>

	et_stop = false;
	std::atomic_bool direct_right = 0;
	emit updateValue();//�������ֵ

	LC.nPulseNum = pulseDistance;//��ʼ����
	LC.Direction = USB1020_PDIRECTION;//��ʼ����Ĭ��Ϊ��
	int pre_direction = LC.Direction;

	double pre_value = 0, begin_value = clear_value;
	while (clear_value < et_threshold && clear_value>10) {//
		pre_value = clear_value;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		speedCheck();//�����ٶȼ�麯�����ٶ�Ϊ0�󣬻����ִ��
		emit updateValue();//����������
		while ((pre_value - clear_value) == 0) {
			emit updateValue();
			emit showNowLocation(USB1020_ReadLP(hDevice, USB1020_XAXIS), USB1020_ReadLP(hDevice, USB1020_YAXIS), USB1020_ReadEP(hDevice, USB1020_ZAXIS));
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		if (abs((pre_value - clear_value) < 2)) {
			LC.nPulseNum = pulseDistance * 4;
		}
		if (clear_value == 0) {//���1�������ܳ������� ��ʱ�Ȼص�ԭλ�ã�Ȼ�󲽳����룬������
			LC.Direction = (pre_direction == USB1020_MDIRECTION) ? USB1020_PDIRECTION : USB1020_MDIRECTION;//������
			pre_direction = LC.Direction;
			USB1020_InitLVDV(hDevice, &DL, &LC);
			USB1020_StartLVDV(hDevice, LC.AxisNum);
			speedCheck();//�ѻص�ԭλ��
			LC.Direction = (pre_direction == USB1020_MDIRECTION) ? USB1020_PDIRECTION : USB1020_MDIRECTION;//������
			pre_direction = LC.Direction;

			LC.nPulseNum /= 2;//��������
		}
		else if ((begin_value - clear_value) > 2) {//�����ȷ�����С�ˣ�˵�������߷���,ֻ�跴����ͺ�
			LC.nPulseNum = pulseDistance;
			LC.Direction = USB1020_MDIRECTION;//������
			pre_direction = LC.Direction;
			if (direct_right == true) break;
		}
		else if ((begin_value - clear_value) < -2) {//�����ȱ���ˣ�˵�������߶���
			direct_right = true;
			//LC.nPulseNum = pulseDistance;
			//LC.nPulseNum -= 2;
		}
		//if (abs(clear_value - et_threshold) < 3) LC.nPulseNum = pulseDistance / 2;
		if (LC.nPulseNum >= pulseDistance * 150) break;
		if (et_stop == true) break;
	}
	//ѭ�������Ӹ���ͣ���Է�ײ��
	USB1020_InstStop(hDevice, LC.AxisNum);
	emit showNowLocation(USB1020_ReadLP(hDevice, USB1020_XAXIS), USB1020_ReadLP(hDevice, USB1020_YAXIS), USB1020_ReadEP(hDevice, USB1020_ZAXIS));
}
/////////////////////////////////�ۺ���/////////////////////////�뽹����
void MotorMove::bounryMove(double journey) {

	//if (now_location<lower_bounry || now_location>higer_bounry)return now_location;
	LC.LV_DV = USB1020_DV;
	LC.PulseMode = USB1020_CPDIR;
	LC.Line_Curve = USB1020_LINE;

	DL.Multiple = 3;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 1000;
	DL.DriveSpeed = 6000;
	LC.AxisNum = USB1020_ZAXIS;
	std::thread([&] {
		LC.Direction = 1;
		LC.nPulseNum = journey / 0.16 / 2;//�����СΪ����֮��
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		long speed = USB1020_ReadCV(hDevice, LC.AxisNum);//��ȡ��ǰ�ٶ�
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//��ͣ��ȡ�ٶ�֪��ͣ����
		}
		emit arrivedBounryBegin();//��������һ���߽�

		LC.Direction = 0;
		LC.nPulseNum = journey / 0.16;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//��ͣ��ȡ�ٶ�֪��ͣ����
		}
		emit arrivedBounryEnd();//������һ���߽�

		//�ص�֮ǰ�ĵ�
		LC.Direction = 1;
		LC.nPulseNum = journey / 0.16 / 2;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);//��ͣ��ȡ�ٶ�֪��ͣ����
		}
		}).detach();

}
//�ߵ�xy�����λ��,Ȼ�����߻���
void MotorMove::moveToBorder(double x_axis_back, double y_axis_back) {
	std::thread([&] {
		IA.Axis1 = USB1020_XAXIS;
		IA.Axis2 = USB1020_YAXIS;
		LD.Line_Curve = USB1020_LINE;

		LD.ConstantSpeed = 0;
		DL.Multiple = 1;
		DL.Acceleration = 4000;
		DL.AccIncRate = 1000;
		DL.StartSpeed = 1000;
		DL.DriveSpeed = 5000;

		LD.n1AxisPulseNum = 50000;
		LD.n2AxisPulseNum = 50000;
		USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);
		USB1020_StartBitInterpolation_2D(hDevice);

		LONG speed1 = 0, speed2 = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);

		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		//����Ӧ�ý�λ�����úã�֮���ߵ�λ�ú�Ҳ��λ�����ú�
		//�ߵ����޺���ݲ���������
		LD.n1AxisPulseNum = x_axis_back;
		LD.n2AxisPulseNum = y_axis_back;
		USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);
		USB1020_StartBitInterpolation_2D(hDevice);

		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		while (speed1 != 0 || speed2 != 0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		emit showNowLocation(USB1020_ReadCV(hDevice, USB1020_XAXIS), USB1020_ReadCV(hDevice, USB1020_YAXIS), USB1020_ReadCV(hDevice, USB1020_ZAXIS));
		}).detach();
}

/////////////////////////////˽�к���//////////////////////////////////////
void MotorMove::setBaseValue() {
	//�������˶�����
	LC1->LV_DV = USB1020_DV;
	LC1->PulseMode = USB1020_CPDIR;
	LC1->Line_Curve = USB1020_LINE;

	DL1->Multiple = 3;
	DL1->Acceleration = 4000;
	DL1->Deceleration = 4000;
	DL1->StartSpeed = 1000;
	DL1->DriveSpeed = 6000;
}
void MotorMove::speedCheck() {
	LONG speed = USB1020_ReadCV(hDevice, LC.Direction);
	while (1) {
		speed = USB1020_ReadCV(hDevice, LC.Direction);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		if (speed == 0)break;
	}
}
