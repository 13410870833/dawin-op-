#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>
#include "LinuxDARwIn.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"


using namespace Robot;


void sighandler(int sig)
{
	exit(0);
}


void* walk_thread(void* ptr)
{
    while(1) {
        int ch = _getch();
        if(ch == 0x20) {
            if(Walking::GetInstance()->IsRunning() == true) {
                MotionManager::GetInstance()->StopLogging();
                Walking::GetInstance()->Stop();
            }
            else {
                MotionManager::GetInstance()->StartLogging();
                Walking::GetInstance()->Start();
            }
        }
    }
    return NULL;
}

int main(void)
{
	printf( "\n===== walking for DARwIn =====\n\n");
        
       //Walking walk =Walking();
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
    CM730 cm730(&linux_cm730);
        printf("========begin=======\n\n");

	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
		return 0;
	}
        
	printf("======next====\n\n");

	minIni* ini = new minIni(INI_FILE_PATH);
	
    Walking::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    MotionManager::GetInstance()->LoadINISettings(ini);

	printf("Press the ENTER key to begin!\n");
	getchar();
       
    int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition[50], wStartPosition[50], wDistance;

    param[JointData::NUMBER_OF_JOINTS * 5]={0};

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition[id] = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition[id] = Walking::GetInstance()->m_Joint.GetValue(id);

		if( wStartPosition[id] > wGoalPosition[id] )
			wDistance = wStartPosition[id] - wGoalPosition[id];
		else
			wDistance = wGoalPosition[id] - wStartPosition[id];

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition[id]);
		param[n++] = CM730::GetHighByte(wGoalPosition[id]);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

  //  Action::GetInstance()->m_Joint.SetEnableBody(true, true); 
  //  Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
  //	MotionManager::GetInstance()->SetEnable(true);
  //	printf("*************************\n\n");
       
    printf("Press the ENTER key to begin!\n");
    getchar();
    printf("Press the SPACE key to start/stop walking.. \n\n");

    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
    MotionManager::GetInstance()->SetEnable(true);

    static const int MAX_FSR_VALUE = 254;
    int left_fsr_x, left_fsr_y, right_fsr_x, right_fsr_y;

    Walking::GetInstance()->LoadINISettings(ini);
    pthread_t thread_t;
    pthread_create(&thread_t, NULL, walk_thread, NULL);




















/*	Walking* walk=Walking::GetInstance();
         
     
     int i=0;
     int startposition[50]={0};
     param[JointData::NUMBER_OF_JOINTS * 5]={0};
     n=0;
     Walking::GetInstance()->X_MOVE_AMPLITUDE = 1.0;
     Walking::GetInstance()->Start();

	while(i<100)
	{
		signal(SIGQUIT, &sighandler);
                
       for(int id=JointData::ID_R_SHOULDER_PITCH;+ id<JointData::NUMBER_OF_JOINTS; id+)
	   {
                 startposition[id]= wGoalPosition[id];
	   }
                 
       walk->Process();

       for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	   {
	        wGoalPosition[id] = walk->m_Joint.GetValue(id);

		if( startposition[id]> wGoalPosition[id] )
			wDistance = startposition[id] - wGoalPosition[id];
		else
			wDistance = wGoalPosition[id] -startposition[id];

	
		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition[id]);
		param[n++] = CM730::GetHighByte(wGoalPosition[id]);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	    } 
	    cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	

        i++;
        n=0;
        usleep(1000000);
	}   */
      printf("=======end=======\n\n");
}



