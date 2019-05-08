#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include "minIni.h"
#include "LinuxDARwIn.h"
#include "vision_deal.h"
#include "opencv2/opencv.hpp"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define INI_FILE_PATH1       "color.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"


/* minTin : ��ini�ļ��м������ò���? LinuxCamera:  ����ͷ��ʼ���������ı�ͻ�ȡͼ��? Mjpg_streamer:  ѹ��YUVͼ��Ϊmjpgͼ��mjpgͼ������ҳ��ʾ*/

#define tracker_color 1

using namespace cv;
using namespace Robot;

char *buf = new char[640 * 480 * 3];
Mat Frame(640, 480, CV_8UC3, buf);

void change_current_dir();     //�ı䵱ǰ����Ŀ¼
void (minIni* ini, const std::string section, int *pData);

void *video_get(void *cap) 
{
	VideoCapture *capture = (VideoCapture*)cap;
	while(true) 
	{
		*capture >> Frame;
	}
}

int main(void)
{
    printf( "\n===== Losers are always in the wrong =====\n\n");

    minIni* ini = new minIni(INI_FILE_PATH);       //����ini�ļ���������ͷ�����趨
    minIni* ini1 = new minIni(INI_FILE_PATH1);     //minTin : ��ini�ļ��м������ò���
	change_current_dir();                          //�ı䵱ǰ����Ŀ¼

	Action::GetInstance()->LoadFile(MOTION_FILE_PATH);  //���ش���Ķ���

    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

	//////////////////// Framework Initialize ��ʼ��///////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);     //��LinuxCM730��������Ϊlinux_cm730
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
	///////////////////////////////////////////////////////////////////////

	while(1)
	{
	    int value=0;
	    usleep(100);
	    if(cm730.ReadByte(CM730::ID_CM,CM730::P_BUTTON,&value,0)==CM730::SUCCESS && value==1)    
	    break; 
	}


	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());  //��ʼ��
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());  ///��ʼ��
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());//��ʼ��
	LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance()); //��ʼ��  ������ ע��ͷ���Ͳ���ģ��,�����м�ʱģ�飨��ȷ������
	motion_timer->Start();																//��ʼ��//
	MotionManager::GetInstance()->LoadINISettings(ini);                            //��ʼ��

///////////////////////////////�Լ���ӵĶ�ȡ�����ǵ�ֵ///////////////////////////////////////////

	printf("GFB_y:");    	if (cm730.ReadWord(CM730::P_GYRO_Y_L, &value, 0)		== CM730::SUCCESS)		printf("%3d\n", value);	else		printf("---");	printf(" GRL_x:");
	if (cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &value, 0) == CM730::SUCCESS)
		printf("%3d", value);
	else
		printf("---");//////////////////////////////////////////////////////////////////////////////////////////////////////	////////////////////////////////�����������ͻ��ٴγ�ʼ������ini�ļ�/////////////////////////////////////////////
	int	firm_ver = 0;
	//int CM730::ReadByte(int id, int address, int *pValue, int *error)
	if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)     //CM730::SUCCESS��CM730.h��public:enum{SUCCESS}   
	{	fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
		exit(0);
	}   

	if(0 < firm_ver && firm_ver < 27)         
	{   
#ifdef MX28_1024
		Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
		fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }   
	else if(27 <= firm_ver)
	{   
#ifdef MX28_1024
		fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
		fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
		exit(0);
#else
		Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
	}   
   else
		exit(0);
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);    
    Head::GetInstance()->LoadINISettings(ini);
	MotionManager::GetInstance()->SetEnable(true); //

	Action::GetInstance()->Start(16);    //Start(1) ����1���е�mp3����
	while(Action::GetInstance()->IsRunning()) usleep(8*1000);  //�����ж���ǰ��ִ��һ��while����������Ķ�����ֱ��������ʼ�ͽ���ִ�л���������Ķ���
	Action::GetInstance()->m_Joint.SetEnableBody(false, false);                  //������µ�

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);                  //��ͷ������ϵ��ʼ��
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);         //���������ϵ��ʼ��
	/*  ��JointData.h��void SetEnableBodyWithoutHead(bool enable, bool exclusive);
		void JointData::SetEnableBodyWithoutHead(bool enable, bool exclusive)
		{
			SetEnableRightArmOnly(enable, exclusive);
			SetEnableLeftArmOnly(enable, exclusive);
			SetEnableRightLegOnly(enable, exclusive);
			SetEnableLeftLegOnly(enable, exclusive);
		}
	*/
	MotionManager::GetInstance()->SetEnable(true);   //��ʼ��
	
	vision_deal vd;
	pthread_t video_id;

	if(0 != pthread_create(&video_id, NULL, video_get, &vd.cap))   //�����߳�
	printf("Pthread can not be created\n");

	LoadData(ini1, std::string("Red"), &(vd.hsv_RectOut[0]));     //������Ŀ¼�µı���õ�hsvֵ
	LoadData(ini1, std::string("Blue"), &(vd.hsv_RectIn[0]));     //������Ŀ¼�µı���õ�hsvֵ

while(1)
{
	Point a;
	Point2D pos;
	int count = 0;
	bool forward_flag = false;
	bool backward_flag = false;
	float err_angle;
	int Mode = 0;

	bool key02 = false;
	int accumulate = 0;

	Head::GetInstance() ->MoveByAngle(0,40);//40     
	/*
		void Head::MoveByAngle(double pan, double tilt)
	{
		m_PanAngle = pan;
		m_TiltAngle = tilt;

		CheckLimit();  //������ֹ���ת������λ��
	}*/


    while(1)
    {
		vd.take_photo(&Frame)��/*    void vision_deal::take_photo(Mat* Frame) 
									{
										resize(*Frame, frame, Size(320, 240)); //��ȡ����ͷһ֡�Ĵ�С
										Point2f po = Point2f(160,120);         //��ȡͼƬ���ĵ�  Ϊpo

										Mat rot_mat = getRotationMatrix2D (po,180,1.0);     //  #Point2f center����ʾ��ת�����ĵ㣬#double angle����ʾ��ת�ĽǶȣ�#double scale��ͼ����������
										warpAffine(frame,frame,rot_mat,frame.size());   //����任  #����ͼ��    #���ͼ��     #�ߴ���sizeָ����ͼ��������ԭͼ��һ��   #�任����    #ָ��ͼ������ߴ�
									}*/
		a = vd.rect_mark(Mode,&err_angle);      //     Point vision_deal::rect_mark(int Mode,float *err_angle) ��һ����Ҫȥvision_deal.cpp����rect_mark ��Ա
		pos.X = a.x;
		pos.Y = a.y;

		float pan_angle =0;
		pan_angle = Head::GetInstance()-> GetPanAngle();
//		printf("pan_angle = %f\t",pan_angle);

		float tite_angle = 0;
		tite_angle = Head::GetInstance()-> GetTiltAngle();
//		printf("tite_angle = %f\n",tite_angle);

#ifdef tracker_color   //�궨����

		if((a.x !=-1) && (a.y !=-1))
		{

			if(count ==10)
			{
				sleep(1);
				count++;
			}
			
			else if(count > 10)
			{
				while(!key02)
				{
					cm730.WriteWord(CM730::ID_CM,CM730::P_LED_HEAD_L,CM730::MakeColor(0,0,255),0);
					int value=0;
					usleep(100);
					if(cm730.ReadByte(CM730::ID_CM,CM730::P_BUTTON,&value,0)==CM730::SUCCESS && value==2)    
					{
						key02 = true;
						break;
					}
				}

				if(backward_flag != true)
				{
					forward_flag = true;
				}
				tracker.Process(pos);
			}

			else
			{
				count++;
			}
		}

		if(forward_flag)
		{
	    int value=0;
	    usleep(100);
	    if(cm730.ReadByte(CM730::ID_CM,CM730::P_BUTTON,&value,0)==CM730::SUCCESS && value==1)    
	    break; 
			if((a.x != -1) && (a.y != -1))
			{
				follower.Process(tracker.ball_position);
				cm730.WriteWord(CM730::ID_CM,CM730::P_LED_HEAD_L,CM730::MakeColor(0,255,0),0);
			}
			else
			{
				cm730.WriteWord(CM730::ID_CM,CM730::P_LED_HEAD_L,CM730::MakeColor(255,0,0),0);
				follower.Process(Point2D(0,0));
			}
//near flag ready to stop.
/*	
			if(tite_angle < 20)//24
			{
				Walking::GetInstance()->X_MOVE_AMPLITUDE -= 10 ;//3
				//Walking::GetInstance()->HIP_PITCH_OFFSET = 13;
				if(tite_angle < 16.0)//18
				{
				    printf("reach the finishing line! begin to backward! *******************************\n");
					backward_flag = true;
					forward_flag = false;
					Mode=1;
		//			Walking::GetInstance()->X_MOVE_AMPLITUDE = 0.0;
					Walking::GetInstance()-> Stop();
					usleep(1000000);
					Walking::GetInstance()->HIP_PITCH_OFFSET = 13;
				}
			}
*/			
		}
	#endif

	}

	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0.0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0 ;
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()-> Stop();
}

	return 0;
}
  
void LoadData(minIni* ini, const std::string section, int *pData)
{
	int value=-2;
	if((value = ini->geti(section,"h_min", INVALID_VALUE)) != INVALID_VALUE) 
		*pData++ = value;
	if((value = ini->geti(section,"h_max", INVALID_VALUE)) != INVALID_VALUE)
		*pData++ = value;
	if((value = ini->geti(section,"s_min", INVALID_VALUE)) != INVALID_VALUE)
		*pData++ = value;
	if((value = ini->geti(section,"s_max", INVALID_VALUE)) != INVALID_VALUE)
		*pData++ = value;
	if((value = ini->geti(section,"v_min", INVALID_VALUE)) != INVALID_VALUE)
		*pData++ = value;
	if((value = ini->geti(section,"v_max", INVALID_VALUE)) != INVALID_VALUE)
		*pData++ = value;
}

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    chdir(dirname(exepath));
}
