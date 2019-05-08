/*
 * ColorFinder.cpp
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#include <stdlib.h>

#include "ColorFinder.h"
#include "ImgProcess.h"
#include <math.h>

using namespace Robot;

ColorFinder::ColorFinder() :
        m_center_point(Point2D()),
        m_hue(356),
        m_hue_tolerance(15),
        m_min_saturation(50),
        m_min_value(10),
        m_min_percent(0.007),
        m_max_percent(30.0),
        color_section(""),
        m_result(0)
{ }

ColorFinder::ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per) :
        m_hue(hue),
        m_hue_tolerance(hue_tol),
        m_min_saturation(min_sat),
        m_min_value(min_val),
        m_min_percent(min_per),
        m_max_percent(max_per),
        color_section(""),
        m_result(0)
{ }

ColorFinder::~ColorFinder()
{
    // TODO Auto-generated destructor stub
}

void ColorFinder::Filtering(Image *img)
{
    unsigned int h, s, v;
    int h_max, h_min;

    if(m_result == NULL)
        m_result = new Image(img->m_Width, img->m_Height, 1);  

   /* h_max = m_hue + m_hue_tolerance;  
    h_min = m_hue - m_hue_tolerance;
    if(h_max > 360)
        h_max -= 360;
    if(h_min < 0)
        h_min += 360;*/

    for(int i = 0; i < img->m_NumberOfPixels; i++)
    {
        	h = (img->m_ImageData[i*img->m_PixelSize + 0] << 8) | img->m_ImageData[i*img->m_PixelSize + 1];
        	s =  img->m_ImageData[i*img->m_PixelSize + 2];
        	v =  img->m_ImageData[i*img->m_PixelSize + 3];

	
        //if( h > 360 )
          //  h = h % 360;
		if((int)v > m_min_value)// && (int)v <m_min_saturation)
		{
			m_result->m_ImageData[i] = 1;
		}
		else
		{
			m_result->m_ImageData[i] = 0;
		}

       /* if( ((int)s > m_min_saturation) && ((int)v > m_min_value) )//  s  v ¼«ÏÞ
        {
            if(h_min <= h_max)
            {
                if((h_min < (int)h) && ((int)h < h_max))
                    m_result->m_ImageData[i]= 1;
                else
                    m_result->m_ImageData[i]= 0;
            }
            else
            {
                if((h_min < (int)h) || ((int)h < h_max))
                    m_result->m_ImageData[i]= 1;
                else
                    m_result->m_ImageData[i]= 0;
            }
        }
        else
        {
            m_result->m_ImageData[i]= 0;
        }
		*/
    }
}

void ColorFinder::Depart(int i, unsigned char* ob, bool* result, int* total, int* border)  //  ture Spilit
{
	(*total)++;
	(*border)++;
	ob[i] = 0;
	if(ob[i-m_result->m_Width] == 1)
	{
		result[i] = 1;
		Depart(i - m_result->m_Width, ob, result, total, border);
	}
	else
	{
		(*border)++;
		//result[i] = 2;
	//	return ;
	}
	if(ob[i+m_result->m_Width] == 1)
	{
		result[i] = 1;
		Depart(i + m_result->m_Width, ob, result, total, border);
	}
	else
	{
		(*border)++;
		//result[i] = 2;
	//	return ;
	}
	if(ob[i-1] == 1)
	{
		result[i] = 1;
		Depart(i-1, ob, result, total, border);
	}
	else
	{
		(*border)++;
		//result[i] = 2;
	//	return ;
	}
	if(ob[i+1] == 1)
	{
		result[i] = 1;
		Depart(i+1, ob, result, total, border);
	}
	else
	{
		(*border)++;
		//result[i] = 2;
	//	return ;
	}
	
}

void ColorFinder::Link(Image* img)  //  find the gread module
{
	bool *First = new bool[img->m_NumberOfPixels];
	bool *Second = new bool[img->m_NumberOfPixels];
	int  old_count = 0;

	Filtering(img);

	ImgProcess::Erosion(m_result);
	ImgProcess::Erosion(m_result);
	ImgProcess::Dilation(m_result);
	ImgProcess::Dilation(m_result);
	ImgProcess::Dilation(m_result);

	for(int j=0; j< img->m_NumberOfPixels; j++)
	{	
		if(m_result->m_ImageData[j]==1)
		{	
			for(int k=0; k<img->m_NumberOfPixels; k++)
			{
				First[k] = 0;
			}
			int ne_count= 0;	int total=0;
			Depart(j, m_result->m_ImageData, First, &ne_count, &total);
			if(ne_count > old_count)
			{	
				old_count =  ne_count;
				for(int i=0; i<m_result->m_NumberOfPixels; i++)
				{	
					Second[i] = First[i];
				}
				
			}
		}
	}
	for(int i=0; i<img->m_NumberOfPixels; i++)
	{
		m_result->m_ImageData[i] = Second[i];
	}			
	delete First;
	delete Second;
}

/*void ColorFinder::LinkCircle(Image* img, float* posX, float* posY) //  border + circle
{
	int* First = new int[img->m_NumberOfPixels];
	bool *Second = new bool[img->m_NumberOfPixels];
	bool *Third = new bool[img->m_NumberOfPixels];
	int  old_count = 0;

	Filtering(img);

	ImgProcess::Erosion(m_result);
	ImgProcess::Dilation(m_result);
	ImgProcess::Erosion(m_result);
	ImgProcess::Dilation(m_result);
	ImgProcess::Dilation(m_result);

	int p =0;
	for(int j=0; j< img->m_NumberOfPixels; j++)
	{	
		if(m_result->m_ImageData[j]==1)
		{	
			//printf("findcontours is &d\n:", p++);
			for(int k=0; k<img->m_NumberOfPixels; k++)
			{
				First[k] = 0;
			}
			int ne_count= 0; int total = 0;
			Depart(j, m_result->m_ImageData, First, &ne_count, &total);
			//printf("                %d               ",total);
			if(ne_count > old_count)
			{	
				printf("                  1               \n");
				old_count = ne_count;
				//printf("The number of contours is:%d\n", old_count);
				//float sumX = 0, sumY = 0;
				for(int i=0; i<m_result->m_NumberOfPixels; i++)
				{//printf("\n");
					if(First[i] == 2)
					{
						sumX += i % m_result->m_Width;
						sumY += i / m_result->m_Width;
					//	printf("%f\t%f\n:",sumX,sumY);
					}
				}
				(*posX) = sumX / total;
				(*posY) = sumY / total;
			*/	
				//for(int i=0; i<m_result->m_NumberOfPixels; i++)
				//{
					//if(First[i] == 2)
					//{
						//float y2 = (i/m_result->m_Width - (*posY))*(i/m_result->m_Width - (*posY));
						//float x2 = (i%m_result->m_Width - (*posX))*(i%m_result->m_Width - (*posX));
						//if(sqrt(x2 + y2) == 50)
						//{
				//			Second[i] = First[i];
						//}
					//else
					//{
					//	Second[i] = 0;
					//}
				//}
/*			}
		}
	}
	printf("                       %d              \n",p);
	for(int i=0; i<m_result->m_NumberOfPixels; i++)
	{
		p++;
		m_result->m_ImageData[i] = Second[i];
	}
	printf("_________________%d                \n",p);
	delete First;
	delete Second;
	delete Third;
}*/

//int ColorFinder::Check_Circle(int i, bool *data)

void ColorFinder::Centerget(Image* img)  //  find the center between two line
{
	Filtering(img);
	for(int j=0; j < img->m_Height; j++)
	{
		for(int i=img->m_Width/2; i < img->m_Width*3/4;i++)   //checek right border
		{
			if(m_result->m_ImageData[j*img->m_Width+i] == 0)
			{
				if(m_result->m_ImageData[j*img->m_Width+i+1] == 1)  
				{
					m_result->m_ImageData[j*img->m_Width+i+1] = 2 ;  
				}
			}	
		}	
		
		for(int i=img->m_Width/2; i > img->m_Width*1/4;i--)   // check left border
		{
			if(m_result->m_ImageData[j*img->m_Width+i] == 0)
			{
				if(m_result->m_ImageData[j*img->m_Width+i-1] == 1)
				{
					m_result->m_ImageData[j*img->m_Width+i-1] = 2 ;
				}
			}
		}
	}
	for(int p=0; p<img->m_Height; p++)  //  draw spilit line 
	{
		for(int i=img->m_Width/2; i<img->m_Width*3/4;i++)
		{
			if(m_result->m_ImageData[p*img->m_Width+i]==2)
			{
				for(int j=img->m_Width/2; j>img->m_Width*1/4;j--)
				{
					if(m_result->m_ImageData[p*img->m_Width+j]==2)
					{
						m_result->m_ImageData[p*img->m_Width+(i+j)/2]=3;
					}
				}
			}
		}
	}
}
void ColorFinder::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, COLOR_SECTION);
}

void ColorFinder::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    if((value = ini->geti(section, "hue", INVALID_VALUE)) != INVALID_VALUE)             m_hue = value;
    if((value = ini->geti(section, "hue_tolerance", INVALID_VALUE)) != INVALID_VALUE)   m_hue_tolerance = value;
    if((value = ini->geti(section, "min_saturation", INVALID_VALUE)) != INVALID_VALUE)  m_min_saturation = value;
    if((value = ini->geti(section, "min_value", INVALID_VALUE)) != INVALID_VALUE)       m_min_value = value;

    double dvalue = -2.0;
    if((dvalue = ini->getd(section, "min_percent", INVALID_VALUE)) != INVALID_VALUE)    m_min_percent = dvalue;
    if((dvalue = ini->getd(section, "max_percent", INVALID_VALUE)) != INVALID_VALUE)    m_max_percent = dvalue;

    color_section = section;
}

void ColorFinder::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, COLOR_SECTION);
}

void ColorFinder::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "hue",              m_hue);
    ini->put(section,   "hue_tolerance",    m_hue_tolerance);
    ini->put(section,   "min_saturation",   m_min_saturation);
    ini->put(section,   "min_value",        m_min_value);
    ini->put(section,   "min_percent",      m_min_percent);
    ini->put(section,   "max_percent",      m_max_percent);

    color_section = section;
}

Point2D& ColorFinder::GetPosition(Image* hsv_img)  //  get the center of a image(y=60-180;x=80-240)
{
    int sum_x = 0, sum_y = 0, count = 0;

	Link(hsv_img);  //  black or write

    ImgProcess::Erosion(m_result);//  (2)
    ImgProcess::Dilation(m_result);  //  (3)
	for(int y = 0; y<180; y++)  //  60 -0
	{
		for(int x = 80; x<240; x++)
		{
			if(m_result->m_ImageData[hsv_img->m_Width*y + x] == 1)
			{
				sum_x += x;
                sum_y += y;
                count++;
			}
		}
	}

    if(count <= (hsv_img->m_NumberOfPixels * m_min_percent / 100) || count > (hsv_img->m_NumberOfPixels * m_max_percent / 100))
    {
        m_center_point.X = -1.0;
        m_center_point.Y = -1.0;
    }
    else
    {
        m_center_point.X = (int)((double)sum_x / (double)count);
        m_center_point.Y = (int)((double)sum_y / (double)count);

////////////////////////////////////////////////////////////////////
		/*for(int i = m_center_point.X; i <240; i++ )
		{
			if(m_result->m_ImageData[hsv_img->m_Width*(int)m_center_point.Y + i] == 1 && m_result->m_ImageData[hsv_img->m_Width*(int)m_center_point.Y + i+1] ==0)
			{
				sum_x = i;
			}
		}
		for(int j = m_center_point.X; j>80; j--)
		{
			if(m_result->m_ImageData[hsv_img->m_Width*(int)m_center_point.Y + j] == 1 && m_result->m_ImageData[hsv_img->m_Width*(int)m_center_point.Y + j-1] ==0)
			{
				sum_y = j;
			}
		}
		if(sum_x - sum_y < 100)
		{
			printf("----------find-----------\n");
		}
		else
		{
			m_center_point.X = -1.0;
			m_center_point.Y = -1.0;
		}*/
////////////////////////////////////////////////////////////////		
    }
	
    for(int y = 20; y <= 180; y++)  //60-20
    {
        for(int x = 80; x <= 240; x++)
        {
			if (x==80 || x==240 || y==20 || y==180)
			{
				m_result->m_ImageData[hsv_img->m_Width *y + x]  = 2;  //  draw the Framework
			}
			else if(x-m_center_point.X<3 && x-m_center_point.X>-3)  //  draw the ten
			{
				m_result->m_ImageData[hsv_img->m_Width *y +x] = 3;
			}
			else if(y-m_center_point.Y<3 && y-m_center_point.Y>-3)
			{
				m_result->m_ImageData[hsv_img->m_Width *y +x] = 3;
			}
            else if(m_result->m_ImageData[hsv_img->m_Width * y + x] == 1)  //  find the object
            {
				m_result->m_ImageData[hsv_img->m_Width *y + x] = 1;
            }
			else
			{
				m_result->m_ImageData[hsv_img->m_Width *y + x] = 0;
			}	
        }
	}
    return m_center_point;
}

/*void ColorFinder::Mark(Image* hsv_img, int Wa, int Wi int Ha, int Hi )  //  draw the framework
{
	
	for(int i = 0; i < 240; i++)
        {
			for(int j = 0; j < 360; j++)
			{
				if (j==Wi || j==Wa || i==Ha || i==Hi)
				{
					m_result->m_ImageData[hsv_img->m_Width *j + i]  = 2;
				}	
            }
        }
}*/
