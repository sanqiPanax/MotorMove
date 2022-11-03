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
		if (x_LP > 0) {//�������0����ת��0��
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
		//����ǰ�������в����ܲ��ܽ����ı�������������ı��ƶ�����������Ծ������ｫ����ȥ��
		LC.nPulseNum = abs(y_LP);
		if (y_LP > 0) {//�������0����ת��0��
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
		//����ǰ�������в����ܲ��ܽ����ı�������������ı��ƶ�����������Ծ������ｫ����ȥ��
		LC.nPulseNum = abs(z_LP);
		if (z_LP > 0) {//�������0����ת��0��
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

	basedThreadSend();//�����̷߳��ͻ����˶���ɵĺ���
}
void MotorMove::triAxis() {
	IA.Axis1 = USB1020_XAXIS;
	IA.Axis2 = USB1020_YAXIS;
	IA.Axis3 = USB1020_ZAXIS;

	//��ʱ�������������ģʽ
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
		unsigned int sign_of_stop = 0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		while (speed1 != 0 || speed2 != 0 || speed3 != 0) {//ֻҪ����һ������ٶȲ�Ϊ0����һֱ��
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
			speed3 = USB1020_ReadCV(hDevice, USB1020_ZAXIS);
		}
		//����û�б�Ҫ����ԭ�������ȫ�����ؼ���
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
/////////////////////////////z���ѭ���˶�����غ���////////////////////////////////////////////////////////////
int zAxisPulse = 0;
int zAxisSteps = 0;
void MotorMove::zAxisLoopMove(int steps, int step_distance) {
	zAxisPulse = step_distance;//��ȫ�ֱ���������ֵ
	zAxisSteps = steps;
	int n = (steps - 1) / 2;
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
	LC.nPulseNum = step_distance*n;
	LC.Direction = 0;
	
	zAxisThreadSend(1);
}
//z����źŷ��ͺ��̴߳���
void MotorMove::zAxisThreadSend(int output) {
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed = 0;
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		}
		emit zAxisLoopMoveComplate(output);
		}).detach();
	LC.Direction = 1;
	LC.nPulseNum = zAxisPulse;
}
void MotorMove::backToMiddlePoint() {
	LC.Direction = 0;
	LC.nPulseNum = (zAxisSteps - 1) / 2 * zAxisPulse;
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed = 0;
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		}
		emit zAxisLoopMoveComplate(0);//����0��ʾ�Ѿ��ص�ԭ��
		}).detach();
}
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////xy�ġ��������˶�/////////////////////////////////////////////
int xLength = 0;
int yWidth = 0;
int xDistance = 0;
int yDistance = 0;
int xyTimes = 1;
int yCount = 0;//�����л�����
void MotorMove::xyAxisMove(int length, int width, int level_distance, int vertical_distance) {
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
void MotorMove::xyAxisThreadSend(int location_x, int location_y) {
	++xyTimes;
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed1 = 0,speed2=0;
		speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
		speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);

		while (speed1!= 0||speed2!=0) {
			speed1 = USB1020_ReadCV(hDevice, USB1020_XAXIS);
			speed2 = USB1020_ReadCV(hDevice, USB1020_YAXIS);
		}
		emit xyAxisMoveComplate(location_x, location_y);
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
	cout << "x�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_XAXIS) << endl;
	cout << "x�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_XAXIS) << endl;

	cout << "y�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_YAXIS) << endl;
	cout << "y�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_YAXIS) << endl;

	cout << "z�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_ZAXIS) << endl;
	cout << "z�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_ZAXIS) << endl;
}

