#include "CameraAim.h"
#include "Subsystems/Chassis.h"
#include "OI.h"
#include "Video.h"

CameraAim::CameraAim(Target_side side)
	: m_tolerance(0)
	, m_side(side)
{
	Requires(ChassisSubsystem::GetInstance());
}

// Called just before this Command runs the first time
void CameraAim::Initialize()
{
	timer.Reset();
	timer.Start();
	m_tolerance = 0;
}

// Called repeatedly when this Command is scheduled to run
void CameraAim::Execute()
{
	if (m_tolerance == 0 || (timer.Get() > 1.75 && fabs(ChassisSubsystem::GetInstance()->GetPIDError()) < m_tolerance)) {
		float turn0 = 0;
		float turn1 = 0;
		float turn = 0;
		size_t nTurns = 0;
		RobotVideo::GetInstance()->mutex_lock();
		nTurns = RobotVideo::GetInstance()->HaveHeading();
		if(nTurns > 0) turn0 = RobotVideo::GetInstance()->GetTurn(0);
		if(nTurns > 1) turn1 = RobotVideo::GetInstance()->GetTurn(1);
		RobotVideo::GetInstance()->mutex_unlock();

		if (nTurns > 1) {
			if (m_side==kRight) turn = turn0 > turn1 ? turn1 : turn0;
			else                turn = turn0 > turn1 ? turn0 : turn1;
		}
		else turn = turn0;

		if (nTurns > 0) {
			m_tolerance = fabs(turn) / 8;
			if (m_tolerance < 1.0) m_tolerance = 1.0;
			ChassisSubsystem::GetInstance()->HoldAngle(turn);
			std::ostringstream oss5;
			oss5 << "Tol: " << m_tolerance;
			SmartDashboard::PutString("DB/String 5", oss5.str());
			timer.Reset();
			timer.Start();
		}
		std::ostringstream oss7;
		oss7 << (m_side==kRight?"R:":"L:") << nTurns << " 0:" << turn0 << " 1:" << turn1;
		SmartDashboard::PutString("DB/String 7", oss7.str());
	}
	std::ostringstream oss6;
	oss6 << "Err: " << ChassisSubsystem::GetInstance()->GetPIDError();
	SmartDashboard::PutString("DB/String 6", oss6.str());

	OI* oi = OI::GetInstance();
	if(oi) {
		// Y-axis positive is down. We want positive - up. Flip it!
		double LSpeed = -oi->stickL->GetY();
		double RSpeed = -oi->stickR->GetY();
		double LMultiplier = (0.5 * oi->stickL->GetZ()) + 0.5;
		double RMultiplier = (0.5 * oi->stickR->GetZ()) + 0.5;
		double moveSpeed = (LSpeed * LMultiplier + RSpeed * RMultiplier) / 2.0;
		moveSpeed *= fabs(moveSpeed); // Square it here so the drive will feel like it's squared
		ChassisSubsystem::GetInstance()->DriveStraight(moveSpeed);
	}

}

// Make this return true when this Command no longer needs to run execute()
bool CameraAim::IsFinished()
{
	return !OI::GetInstance()->stickL->GetTrigger();
}

// Called once after isFinished returns true
void CameraAim::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraAim::Interrupted()
{

}
