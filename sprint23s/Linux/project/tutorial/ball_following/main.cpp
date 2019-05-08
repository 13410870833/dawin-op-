/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <libgen.h>
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
#include "JointData.h"
#include "opencv2/opencv.hpp"

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;


void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Ball following Tutorial for DARwIn =====\n\n");

    change_current_dir();

    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;
	JointData jointData;

	//ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50);
	//yellow_finder->LoadINISettings(ini,"YELLOW");


    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();
	follower.DEBUG_PRINT = true;

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int value;
	while(1)
	{
		if(cm730.ReadWord(CM730::ID_CM, CM730::P_BUTTON, &value, 0) == CM730::SUCCESS && value == 2)
		{
			break;
		}
		usleep(10);
	}
	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

	printf("longrun is ready!\n");
	getchar();
	
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);

	Point2D pos_yellow, pos_red, pos; float posx, posy;
	int Y_value, Z_value;
	double body_Angle;
    while(1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();

        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
		pos = ball_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

		//yellow_finder->Link(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		if(pos.X == -1 && pos.Y == -1)
		{
			tracker.SearchAndTracking(pos);
		}
		else
		{
			cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &Y_value, 0);  //  change
			
			cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &Z_value, 0);  //  change
			body_Angle = atan(512.0 - (double)Y_value)/((double)Z_value - 512.0) * 180/3.1416;
			printf("body_Angle:%f\n",body_Angle);  //  change

			if(body_Angle <0.78 || body_Angle > 0.72)
			{
				tracker.Process(pos);
				follower.Process(tracker.ball_position);
			}
			/*double TiltAngle = jointData.GetAngle(20);
			TiltAngle = (double)39 * tan(TiltAngle*3.14/180);
			double PanAngle = jointData.GetAngle(19);
			PanAngle = TiltAngle * tan(PanAngle*3.14/180);
			printf("                    distance:%f              offset:%f\n",TiltAngle,PanAngle);*/
			
		}

//	pos_yellow =  yellow_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
		
        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(ball_finder->m_result->m_ImageData[i] == 3)
			{
              	rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
			}
			if(ball_finder->m_result->m_ImageData[i] == 2)
			{
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 0;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255;
			}
			if(ball_finder->m_result->m_ImageData[i] == 1)
			{
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
			}
			/*if (yellow_finder->m_result->m_ImageData[i] == 1)
			{
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
				rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
			}*/
		}
		streamer->send_image(rgb_ball);

	}
}
