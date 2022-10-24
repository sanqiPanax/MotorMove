#include "MotorMove.h"

MotorMove::MotorMove(QObject* parent)
{

}
MotorMove::~MotorMove() {

}

//�����˶�
void MotorMove::basedMove(int axis,int function,int pulse) {
	//�жϳ��������
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
	//ִ���깦�ܺ�Ӧ��decadeAxis������⵽�������������
	resetAxis();
}

/////////////////////////////////////�����˶�//////////////////////////////
//ǰ��
void MotorMove::moveForward(int pulse) {
	//ֻ��һ��
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
//����
void MotorMove::moveBack(int pulse) {
	//ֻ��һ��
	if (nums_of_axis == 1) {
		LC.nPulseNum = pulse;
		singleAxis();
	}
	if (nums_of_axis == 2) {

	}
	if (nums_of_axis == 3) {

	}
}
//����
void MotorMove::setZero() {

}
//����
void MotorMove::backToZero() {

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
	//��ʱ�������������������
	LD.Line_Curve = USB1020_LINE;//ֱ�߼���
	LD.ConstantSpeed = USB1020_CONSTAND;//�̶��ٶ�

	DL.Multiple = 10;
	DL.Acceleration = 4000;
	DL.Deceleration = 4000;
	DL.StartSpeed = 2000;
	DL.DriveSpeed = 8000;

	USB1020_InitLineInterpolation_2D(hDevice, &DL, &IA, &LD);//2D�Ĳ�������
	USB1020_StartLineInterpolation_2D(hDevice);
}
void MotorMove::triAxis() {

}
//////////////////////////////////////////////////////////////////


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

///////////////////////////////�̺߳���///////////////////////////////////
//�����˶���������źŷ��ͺ������߳�
void MotorMove::basedThreadSend() {
	std::thread([&] {
		LONG speed1 = 0, speed2 = 0, speed3 = 0;//��������ٶ�
		unsigned int sign_of_stop = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed1 != 0 || speed2 != 0 || speed3 != 0) {//ֻҪ����һ������ٶȲ�Ϊ0����һֱ��
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