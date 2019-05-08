/*
 *   BallTracker.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <math.h>
#include "Head.h"
#include "Camera.h"
#include "ImgProcess.h"
#include "BallTracker.h"
#include "Walking.h"
#include "MotionStatus.h"

using namespace Robot;


BallTracker::BallTracker() :
        ball_position(Point2D(-1.0, -1.0))
{
	NoBallCount = 0;
	count =0;
}

BallTracker::~BallTracker()
{
}

int BallTracker::Search(bool count_reset)
{

        double head_pan_angle = Head::GetInstance()->GetPanAngle();
        double head_tilt_angle =  Head::GetInstance()->GetTiltAngle();
	
	switch(count){
	case 0: 
		 if((TiltTopLimit - head_tilt_angle) > (head_tilt_angle - TiltBottomLimit))
		 {
       		    head_pan_angle += 2;
       		  	if(head_pan_angle > PanLimit){
				Head::GetInstance()->MoveByAngle(PanLimit, TiltBottomLimit);
				count = 1;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, TiltBottomLimit);
		}
		else
		{
       		    head_pan_angle += 2;
       		  	if(head_pan_angle > PanLimit){
				Head::GetInstance()->MoveByAngle(PanLimit, TiltTopLimit);
				count = 1;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, TiltTopLimit);
		
		}
		break;
	case 1:
		if(head_tilt_angle == TiltTopLimit)
		{
			head_pan_angle -= 2;
       		 	if(head_pan_angle < -PanLimit){
			 	Head::GetInstance()->MoveByAngle(-PanLimit, TiltTopLimit);
			 	count = 2;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, TiltTopLimit);
		}
		else
		{
			head_pan_angle -= 2;
       		 	if(head_pan_angle < -PanLimit){
			 	Head::GetInstance()->MoveByAngle(-PanLimit, TiltBottomLimit);
			 	count = 2;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, TiltBottomLimit);
		
		}
		break;
		
	case 2:
		if(head_tilt_angle == TiltTopLimit)
		{
			head_pan_angle += 2; head_tilt_angle -= 1;
       		 	if(head_pan_angle > PanLimit || head_tilt_angle < TiltBottomLimit){
			 	Head::GetInstance()->MoveByAngle(PanLimit, TiltBottomLimit);
			 	count = 1;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, head_tilt_angle);
		}
		else
		{
			head_pan_angle += 2;  head_tilt_angle += 1;
       		 	if(head_pan_angle  > PanLimit || head_tilt_angle > TiltTopLimit){
			 	Head::GetInstance()->MoveByAngle(PanLimit, TiltTopLimit);
			 	count = 1;}
        	 	else 
				Head::GetInstance()->MoveByAngle(head_pan_angle, head_tilt_angle);
		
		}
		break;
	defalut: break;
	}	
	return 0;
}

int BallTracker::SearchAndTracking(Point2D pos)
{
    static int          found_count     = 0;
    static int          not_found_count = 0;
    static const int    max_found_count = 2;
    static const double max_x_offset    = 3.0;

    if(pos.X < 0 || pos.Y < 0)
    {
        // NOT Found
        ball_position.X = -1.0;
        ball_position.Y = -1.0;
        if(NoBallCount < NoBallMaxCount)
        {
            Head::GetInstance()->MoveTracking();
            NoBallCount++;
        }
        else
        {
            // Search
            Head::GetInstance()->InitTracking();
	    Search(false);

        }

        if(not_found_count > NotFoundMaxCount)
            return -1;
        else
            not_found_count++;
            
        return 0;
    }
    else
    {
        // Found
        not_found_count = 0;
        NoBallCount = 0;
        Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
        Point2D offset = pos - center;
        offset *= -1; // Inverse X-axis, Y-axis
        offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
        offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
        ball_position = offset;
        Head::GetInstance()->MoveTracking(ball_position);

        double head_pan_angle = Head::GetInstance()->GetPanAngle();
        if((-max_x_offset < offset.X && offset.X < max_x_offset) ||
           (head_pan_angle == Head::GetInstance()->GetLeftLimitAngle() || head_pan_angle == Head::GetInstance()->GetRightLimitAngle()))
        {
            //fprintf(stderr, "[SearchAndTracking] FOUND!! \n");
            if(found_count > max_found_count)
            {
                found_count = 0;
                return 1;
            }
            else
                found_count++;
        }
        else
            found_count = 0;
            
        return 1;
    }
}

void BallTracker::Process(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->MoveTracking();
			NoBallCount++;
		}
		else
			Head::GetInstance()->InitTracking();
	}
	else
	{
		NoBallCount = 0;
//		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/3);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTracking(ball_position);
	}
}

