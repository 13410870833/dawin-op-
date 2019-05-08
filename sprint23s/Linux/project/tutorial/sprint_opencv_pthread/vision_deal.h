#ifndef _VISION_DEAL_H
#define _VISION_DEAL_H

#include "opencv2/opencv.hpp"
#include <math.h>

using namespace cv;

namespace Robot
{
	class vision_deal
	{
		private:
			Mat frame;

			Rect left_area ;
			Rect right_area;
			Rect left_top;
			Rect right_top;
			float ending;
			int NoMark_count;
			int findMark_count;
			bool finded_mark;
			VideoWriter writer;

		public:
			VideoCapture cap;
			Mat result_frame;
			int hsv_green[6];
			int hsv_white[6];
			int hsv_red[6];
			int hsv_blue[6];
			int hsv_RectOut[6];
			int hsv_RectIn[6];
			int hsv_ground[6];
			Point result;
			vision_deal();
			~vision_deal();
			Rect Aim_Min_Rect;
			
			void take_photo(Mat*);
			void trackbar(char* windowname,int* hsv);
			vector<vector<Point> > getContour(Mat image);
			bool judge_retangle(vector <Point2f> corners);
			int find_max_contours(vector <vector<Point> > contours);
			int judge_ending(Mat img,Rect area);//判断结束
			int searchobject(vector <Point2f> &M,vector <Point2f> &N);//寻找物体
			Point rect_mark(int Mode,float* err_angle );//矩形标记
			vector<Point2f> find_corners(Mat image,vector<vector <Point> > contours , int idx);//寻找角
			float runing_thread(Rect floor);//
	};
}

#endif

