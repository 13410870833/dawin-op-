/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <string.h>

#include "Camera.h"
#include "ImgProcess.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "Image.h"

#define INI_FILE_PATH       "config.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Camera Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->LoadINISettings(ini);
	Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	int i = 0, sum = 0;
    while(1)
    {
        LinuxCamera::GetInstance()->CaptureFrame();//  ×¥Ö¡
		memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
		

		for(i;i < rgb_ball->m_NumberOfPixels; i++)
		{
			if(i % rgb_ball->m_Width == 80 ||i % rgb_ball->m_Height == 240 || (i >= 180 * rgb_ball->m_Width + 80 && i <= 180 * rgb_ball->m_Width +240))
			{
				if(i / rgb_ball->m_Width <= 180)
				{
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
					rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
					sum += i;
				}
			}
		}
		printf("%d\n",sum);
        streamer->send_image(rgb_ball);  //  ´«ËÍYUVÍ¼Ïñ
    }

    return 0;
}
