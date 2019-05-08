#include <unistd.h>
#include <libgen.h>

#include "opencv2/opencv.hpp"
#include "stdio.h"
#include "vision_deal.h"


using namespace cv;
using namespace Robot;
/*
#define show_image 1 
#define show_trackbar 1
*/

vision_deal::vision_deal()		//打开摄像头
{
	cap.open(0);
	if(!cap.isOpened())
		printf("can not open camera!\n");
	
	result.x = -1.0;
	result.y = -1.0;
}

vision_deal::~vision_deal()   //释放
{
	cap.release();
}

void vision_deal::take_photo(Mat* Frame) 
{
	resize(*Frame, frame, Size(320, 240)); //获取摄像头一帧的大小
	Point2f po = Point2f(160,120);         //获取图片中心点  为po

	Mat rot_mat = getRotationMatrix2D (po,180,1.0);     //  #Point2f center：表示旋转的中心点，#double angle：表示旋转的角度，#double scale：图像缩放因子
	warpAffine(frame,frame,rot_mat,frame.size());   //仿射变换  #输入图像    #输出图像     #尺寸由size指定，图像类型与原图像一致   #变换矩阵    #指定图像输出尺寸
}

void vision_deal::trackbar(char* windowname,int* hsv)  //创建滑动条hsv
{
	namedWindow(windowname);
	cvCreateTrackbar("LH",windowname,hsv++,180);
	cvCreateTrackbar("HH",windowname,hsv++,180);
	cvCreateTrackbar("LS",windowname,hsv++,255);
	cvCreateTrackbar("HS",windowname,hsv++,255);
	cvCreateTrackbar("LV",windowname,hsv++,255);
	cvCreateTrackbar("HV",windowname,hsv,255);
}

vector<vector<Point>>  vision_deal::getContour(Mat image) //取二值图，将所有轮廓的点存放进contours
{
	vector< vector<Point> > contours;
	threshold(image,image,70,255,CV_THRESH_BINARY);    //将阈值内的变白
	findContours(image,contours,CV_RETR_TREE,CHAIN_APPROX_SIMPLE);	
	return contours;
}

int vision_deal::find_max_contours(vector <vector <Point> > contours) //找最大轮廓
{
	int max_idx = 0;
	double max_Area = 0;
	for(int idx = 0; idx <contours.size();idx++)
	{
		double area = contourArea(contours[idx]);    //计算最大轮廓面积double contourArea(InputArray contour, bool oriented=false )

		if(max_Area < area)
		{	
			max_idx = idx ;
			max_Area = area;/////-----------------
		}
	}
	return max_idx;   //返回下标？？？
	
}

bool vision_deal::x(vector<Point2f> corners)///？？？？
{
    long int a[3] = {0,0,0};
	int z=0;
	int retangle_count = 0;     //数角
	if(corners.size()==4)       //如果有四个角
	{   
		for(int i=0;i<corners.size();i++)
	    {   
	         for(int j =0; j<corners.size(); j++)
			 {
	              if(j != i)
				  { 
	                  a[z++] = (corners[i].x-corners[j].x)*(corners[i].x -corners[j].x)+(corners[i].y-corners[j].y)*(corners[i].y-corners[j].y);								}   
	          }   
	          z=0;
			if((a[0]+a[1]>a[2]-20 && a[0]+a[1]<a[2]+20) || (a[0]+a[2]>a[1]-20 && a[0]+a[2]<a[1]+20) || (a[1]+a[2]>a[0]-20 && a[1]+a[2]<a[0]+20))
			retangle_count ++; 
	    }   
	}   
	   if(retangle_count >=2) 
	   return true;
	   else	
	   return false;
}

int vision_deal::judge_ending(Mat img,Rect area)
{
    int count = 0;
    int total_count = 0;

    if(area.x > img.cols || area.x+area.width > img.cols || area.y > img.rows || area.y + area.height >img.rows)
	return 1;

	for(int i = (int)area.y ; i <= (int)area.y+(int)area.height;i++)
	{   
		uchar* data = img.ptr<uchar>(i);

        for(int j = (int)area.x ; j <= (int)area.x+ (int)area.width; j++)
        {   
            total_count++;
            if(data[j] == 255)
				count++;
        }
																				        
    }
    return count;
}

int vision_deal::searchobject(vector <Point2f> &M, vector <Point2f> &N)  //找物体
{
	int num = -1;
	float temp_distance;           //距离
	float min_distance = 102400;   //最小距离

	Point p1, p2;
	for(int i = 0;i<M.size();i++)
	{
		for(int j = 0;j<N.size();j++)
		{
			temp_distance = (M[i].x - N[j].x) * (M[i].x - N[j].x) + (M[i].y - N[j].y) * (M[i].y - N[j].y);
			if(min_distance > temp_distance) 
			{
				min_distance = temp_distance;
				num = i;
				p1 = M[i];
				p2 = N[j];
			}
		}
	}	

	if(min_distance > 100)
	{
		num = -1;
	}
	else if(min_distance != 102400) 
	{ 
		result.x = (int)(p1.x + p2.x + 0.5)/2;
		result.y = (int)(p1.y + p2.y + 0.5)/2;
	}
	printf("Aim Point: (%3d,%-3d) Distance: %-3.f\n", (int)(result.x), (int)(result.y), min_distance);
	return num;
}

Point vision_deal::rect_mark(int Mode,float *err_angle)
{
	result.x = -1;
	result.y = -1;

	Mat image_out = frame.clone();
	Mat temp_frame = frame.clone();

	cvtColor(image_out,image_out,CV_BGR2HSV);
	inRange(image_out,Scalar(hsv_RectOut[0],hsv_RectOut[2],hsv_RectOut[4]),Scalar(hsv_RectOut[1],hsv_RectOut[3],hsv_RectOut[5]),image_out);
	Mat element = getStructuringElement(MORPH_RECT,Size(5,5));   //getStructuringElement返回矩形形状和尺寸的结构元素。##第二个参数为内核
	//morphologyEx(image_out,image_out,MORPH_OPEN,element);
	morphologyEx(image_out,image_out,MORPH_CLOSE,element);    //第三个参数为闭运算 :先膨胀，再腐蚀，可清除小黑点
	//erode(image_out,image_out,element);
	//dilate(image_out,image_out,element);
#ifdef show_image
	#ifdef show_trackbar 
		trackbar("Outer Contour",hsv_RectOut);       //void vision_deal::trackbar(char* windowname,int* hsv)
	#else
		namedWindow("Outer Contour");
	#endif
		imshow("Outer Contour",image_out);    ///显示的是一个二值图
		waitKey(1);
#endif

	Mat image_in = frame.clone();

	cvtColor(image_in,image_in,CV_BGR2HSV);
	inRange(image_in,Scalar(hsv_RectIn[0],hsv_RectIn[2],hsv_RectIn[4]),Scalar(hsv_RectIn[1],hsv_RectIn[3],hsv_RectIn[5]),image_in);
	Mat element_in = getStructuringElement(MORPH_RECT,Size(5,5));
	//morphologyEx(image_in,image_in,MORPH_OPEN,element);
	morphologyEx(image_in,image_in,MORPH_CLOSE,element);
	//dilate(image_out,image_out,element);
	//erode(image_out,image_out,element);

#ifdef show_image
	#ifdef show_trackbar
		trackbar("Inner Countour",hsv_RectIn);
	#else
		namedWindow("Inner Countour");
	#endif
		imshow("Inner Countour",image_in);
		waitKey(1);
#endif

	vector <vector <Point> > contours_out_rect;	
	vector <vector <Point> > contours_in_rect;	

	contours_out_rect = getContour(image_out);//image the bule colorRect in left
	contours_in_rect = getContour(image_in);

	vector <Moments> mu_out(contours_out_rect.size());
	vector <Moments> mu_in(contours_in_rect.size());

	for(int i =0;i< contours_out_rect.size();i++)
	{
		 Moments mu = moments(contours_out_rect[i],false);
		 printf("Outer Contour Area = %.2f\n",mu.m00);
		 if(mu.m00 > 200 || mu.m00 < 6000)  mu_out[i] = mu;
		// if(mu.m00 < 1000)  mu_out[i] = mu;
	}
	if(mu_out.size() <1 )	 return Point(-1,-1);	

	for(int i=0; i<contours_in_rect.size(); i++)
	{
		
		Moments mu  = moments(contours_in_rect[i],false);	
		 printf("Inner Contour Area = %.2f\n",mu.m00);
		if(mu.m00 > 50 || mu.m00 < 3500)  mu_in[i] = mu;
		//if(mu.m00 < 420)  mu_in[i] = mu;
	}
	if(mu_in.size() < 1)  return Point(-1,-1);
	
	vector <Point2f> mc_out(contours_out_rect.size());
	vector <Point2f> mc_in(contours_in_rect.size());

	printf("\nOut Side: >> ");
	for(int i=0; i<contours_out_rect.size();i++)
	{
		mc_out[i] = Point2f(mu_out[i].m10/mu_out[i].m00,mu_out[i].m01/mu_out[i].m00);	
		printf("(%3d,%-3d)  ", (int)mc_out[i].x, (int)mc_out[i].y);
	}

	printf("\n In Side: >> ");
	for(int i=0; i<contours_in_rect.size();i++)
	{
		mc_in[i] = Point2f(mu_in[i].m10/mu_in[i].m00,mu_in[i].m01/mu_in[i].m00);	
		printf("(%3d,%-3d)  ", (int)mc_in[i].x, (int)mc_in[i].y);
	}
	printf("\n");
	
	int num = searchobject(mc_out, mc_in);

	double r = -1;

#ifdef show_image
	if(result.x >= 0 && result.y >= 0)	
	{
		printf("Num: %d\n", num);
		r = ((contours_out_rect[num][0].x - result.x) * (contours_out_rect[num][0].x - result.x) + (contours_out_rect[num][0].y - result.y) * (contours_out_rect[num][0].y - result.y));
		r = sqrt(r) + 10.0;
		if(num != -1 && r != -1) {
			circle(frame, result, (float)r, Scalar(255), 2);//圈出目标
		}
	}

		namedWindow("result");
		imshow("result", frame);
		waitKey(1);
#endif

	return result;
}

