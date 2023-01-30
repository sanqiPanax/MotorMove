#ifndef BUILD_STATIC
#define BUILD_STATIC
#endif // !BUILD_STATIC
#include "MotorMove.h"

MotorMove::MotorMove(QObject* parent)
{
	hDevice = USB1020_CreateDevice(0);

	USB1020_PulseOutMode(hDevice,USB1020_XAXIS, USB1020_CPDIR, 0, 0);
	USB1020_PulseOutMode(hDevice,USB1020_YAXIS, USB1020_CPDIR, 0, 0);
	USB1020_PulseOutMode(hDevice,USB1020_ZAXIS, USB1020_CPDIR, 0, 0);
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
		}).detach();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
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
		return ;
	}
	std::thread([&] {
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		LONG speed = 0;
		speed = USB1020_ReadCV(hDevice, LC.AxisNum);
		while (speed != 0) {
			speed = USB1020_ReadCV(hDevice, LC.AxisNum);
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		emit zAxisLoopMoveComplate();
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
void MotorMove::xyAxisThreadSend() {
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
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		emit xyAxisMoveComplate();
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
	
	std::cout << "x�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_XAXIS) << std::endl;
	std::cout << "x�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_XAXIS) << std::endl;

	std::cout << "y�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_YAXIS) << std::endl;
	std::cout << "y�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_YAXIS) << std::endl;

	std::cout << "z�ᵱǰ�߼�λ�ã�" << USB1020_ReadLP(hDevice, USB1020_ZAXIS) << std::endl;
	std::cout << "z�ᵱǰʵ��λ�ã�" << USB1020_ReadEP(hDevice, USB1020_ZAXIS) << std::endl;
}
//�ı�ȫ�ֱ���clear_point_value��ֵ
void MotorMove::changeChearPointValue(double value) {
	clear_point_value = value;
}
//�ƶ�����������λ��
void MotorMove::moveToMostClearPoint(int large_range, int small_range) {
	emit startMeasure();//������ͷ��ʼ����
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
	LC.nPulseNum = 10;
	bool count = true;
	LC.Direction = count;
	int time = 1;//��һ���ط�ѭ������ξ���ֹ

	double pre_value = clear_point_value;
	double now_value = clear_point_value;
	while (clear_point_value < large_range) {
		pre_value = now_value;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		//ʱ��
		now_value = clear_point_value;
		//std::cout << USB1020_ReadLP(hDevice, USB1020_ZAXIS)<<std::endl;
		if (now_value < pre_value) {
			count = !count;
		}
		else if (now_value == pre_value) {
			count = !count;
			++time;
		}
		LC.Direction = count;
		if (time > 1000)break;
	}
	LC.nPulseNum = 5;
	time = 1;
	while (clear_point_value< small_range) {
		pre_value = now_value;
		USB1020_InitLVDV(hDevice, &DL, &LC);
		USB1020_StartLVDV(hDevice, LC.AxisNum);
		//ʱ��
		now_value = clear_point_value;
		//std::cout << USB1020_ReadLP(hDevice, USB1020_ZAXIS) << std::endl;
		if (now_value < pre_value) {
			count = !count;
		}
		else if (now_value == pre_value) {
			count = !count;
			++time;
		}
		LC.Direction = count;
		if (time > 1000)break;
	}
}