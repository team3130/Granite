#include "CameraTest.h"
#include "OI.h"

CameraTest::CameraTest()
{
	// Use Requires() here to declare subsystem dependencies
	Requires(ChassisSubsystem::GetInstance());
}

// Called just before this Command runs the first time
void CameraTest::Initialize()
{
	RobotVideo::GetInstance()->Enable();
}

// Called repeatedly when this Command is scheduled to run
void CameraTest::Execute()
{
	OI* oi = OI::GetInstance();
	double moveSpeed = -oi->stickL->GetY();
	double speedMultiplier = (0.5 * oi->stickL->GetZ()) + 0.5;

	float turn = RobotVideo::GetInstance()->GetTurn();
	if(turn > 10) turn = 0.12;
	else if(turn < -10) turn = -0.12;
	else turn = 0;

	ChassisSubsystem::GetInstance()->Drive(moveSpeed * speedMultiplier, turn);

	SmartDashboard::PutNumber("Video Heading", RobotVideo::GetInstance()->GetTurn());
	SmartDashboard::PutNumber("Video Distance", RobotVideo::GetInstance()->GetDistance());
}

// Make this return true when this Command no longer needs to run execute()
bool CameraTest::IsFinished()
{
	return fabs(RobotVideo::GetInstance()->GetTurn()) < 10;
}

// Called once after isFinished returns true
void CameraTest::End()
{
	RobotVideo::GetInstance()->Disable();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraTest::Interrupted()
{
	End();
}
