#include "CameraAim.h"
#include "Subsystems/Chassis.h"
#include "OI.h"
#include "Video.h"

CameraAim::CameraAim()
	: m_tolerance(0)
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
	if (m_tolerance == 0 || fabs(ChassisSubsystem::GetInstance()->GetPIDError()) < m_tolerance) {
		float turn = 0;
		bool got_turn = false;
		RobotVideo::GetInstance()->mutex_lock();
		got_turn = RobotVideo::GetInstance()->HaveHeading();
		if(got_turn) turn = RobotVideo::GetInstance()->GetTurn();
		RobotVideo::GetInstance()->mutex_unlock();

		if (got_turn) {
			double newAngle  = turn;
			m_tolerance = fabs(newAngle) / 4;
			ChassisSubsystem::GetInstance()->HoldAngle(newAngle);
			std::ostringstream oss5;
			oss5 << "Tol: " << m_tolerance;
			SmartDashboard::PutString("DB/String 5", oss5.str());
			std::ostringstream oss6;
			oss6 << "New: " << newAngle;
			SmartDashboard::PutString("DB/String 6", oss6.str());
			timer.Reset();
			timer.Start();
		}
	}

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
