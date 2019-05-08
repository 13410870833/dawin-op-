/*
 *   BallFollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "BallFollower.h"
#include "MotionStatus.h"


using namespace Robot;


BallFollower::BallFollower()
{
	m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
	m_KickBallMaxCount = 10;
	m_KickBallCount = 0;

	m_KickTopAngle = -5.0;
	m_KickRightAngle = -30.0;
	m_KickLeftAngle = 30.0;


	m_FollowMaxFBStep = 45.0;  //45
    m_FollowMinFBStep = 43.0;	//43
	m_FollowMaxRLTurn = 8.0;//35
	m_FitFBStep = 3.0;
	m_FitMaxRLTurn = 8.0;//35
	m_UnitFBStep = 1.0;//initial velocity 1.0
	m_UnitRLTurn = 1.0;

	m_GoalFBStep = 0;
	m_GoalRLTurn = 0;
	m_FBStep = 0;
	m_RLTurn = 0;
	DEBUG_PRINT = false;
	KickBall = 0;
	pan = 0; 
}

BallFollower::~BallFollower()
{
}

void BallFollower::Process(Point2D ball_pos)
{
	if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

    if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
    {
		KickBall = 0;

		if(m_NoBallCount > m_NoBallMaxCount)
		{
			// can not find a ball
			m_GoalFBStep = 0;
			m_GoalRLTurn = 0;
			Head::GetInstance()->MoveToHome();

			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL]");
		}
		else
		{
			m_NoBallCount++;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[NO BALL COUNTING(%d/%d)]", m_NoBallCount, m_NoBallMaxCount);
		}
    }
    else
    {
		m_NoBallCount = 0;		

		 pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();
		double pan_percent = pan / pan_range;

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();		
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;
		double tilt_percent = (tilt - tilt_min) / tilt_range;
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)
		{
			if(tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{
				if(ball_pos.Y < m_KickTopAngle)
				{
					m_GoalFBStep = 0;
					m_GoalRLTurn = 0;

					if(m_KickBallCount >= m_KickBallMaxCount)
					{
						m_FBStep = 0;
						m_RLTurn = 0;						
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK]");

						if(pan > 0)
						{
							KickBall = 1; // Left
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Left");
						}
						else
						{
							KickBall = -1; // Right
							if(DEBUG_PRINT == true)
								fprintf(stderr, " Right");
						}
					}
					else
					{
						KickBall = 0;
						if(DEBUG_PRINT == true)
							fprintf(stderr, "[KICK COUNTING(%d/%d)]", m_KickBallCount, m_KickBallMaxCount);
					}
				}
				else
				{
					m_KickBallCount = 0;
					KickBall = 0;
					m_GoalFBStep = m_FitFBStep;
					m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
					if(DEBUG_PRINT == true)
						fprintf(stderr, "[FIT(P:%.2f T:%.2f>%.2f)]", pan, ball_pos.Y, m_KickTopAngle);
				}
			}
			else
			{
				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = m_FollowMaxFBStep * tilt_percent;  //  ×î´óËÙ¶È
				if(m_GoalFBStep < m_FollowMinFBStep)
				    m_GoalFBStep = m_FollowMinFBStep + 5;  //  ×îÐ¡ËÙ¶È
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
				if(DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
			}
		}
		else  //  ÏÈµ÷ÕûÖÁÖÐÐÄ
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
		}		
	}

	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
	{
		//                        special                             //
		//                                                            //
		if(Walking::GetInstance()->IsRunning() == true)
			Walking::GetInstance()->Stop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			Walking::GetInstance()->Start();			
		}
		else
		{
			//                           special                               //
			if( pan < 25 && pan > -25 )
			{
				if(m_FBStep < m_GoalFBStep)
					m_FBStep += m_UnitFBStep;
				else if(m_FBStep >= m_GoalFBStep )
					m_FBStep -= m_UnitFBStep;  //  »ºËÙÏÂ½µ
				Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
	
				if(m_RLTurn < m_GoalRLTurn)
					m_RLTurn += m_UnitRLTurn;
				else if(m_RLTurn > m_GoalRLTurn)
					m_RLTurn -= m_UnitRLTurn;
				Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			}
			//                          special                               //
			else
			{
				m_GoalFBStep = m_FollowMinFBStep;
				if(m_FBStep < m_GoalFBStep)
					m_FBStep += m_UnitFBStep;
				else if(m_FBStep > m_GoalFBStep)
					m_FBStep -= m_UnitFBStep;
				Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
	
				if(m_RLTurn < m_GoalRLTurn)
					m_RLTurn += m_UnitRLTurn;
				else if(m_RLTurn > m_GoalRLTurn)
					m_RLTurn -= m_UnitRLTurn;
				Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			}

			if(DEBUG_PRINT == true)
			{	
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		
			}	
	}

}
}
//                                     long_run                                       //
