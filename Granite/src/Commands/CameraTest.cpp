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
	RobotVideo::GetInstance()->SetHeadingQueueSize(0);
	RobotVideo::GetInstance()->SetLocationQueueSize(0);
	RobotVideo::GetInstance()->Enable();
}

// Called repeatedly when this Command is scheduled to run
void CameraTest::Execute()
{
	OI* oi = OI::GetInstance();
	double moveSpeed = -oi->stickL->GetY();
	double speedMultiplier = (0.5 * oi->stickL->GetZ()) + 0.5;

	float turn = 0;
	RobotVideo::GetInstance()->mutex_lock();
	bool got_turn = RobotVideo::GetInstance()->HaveHeading();
	if(got_turn) turn = RobotVideo::GetInstance()->GetTurn();
	RobotVideo::GetInstance()->mutex_unlock();

	if(got_turn)
		SmartDashboard::PutNumber("Video Heading", turn);
	else
		SmartDashboard::PutNumber("Video Heading", -999);
	//SmartDashboard::PutNumber("Video Distance", RobotVideo::GetInstance()->GetDistance());

	if(turn > 10) turn = turn/700 + 0.2;
	else if(turn < -10) turn = turn/700 - 0.2;
	else turn = 0;

	ChassisSubsystem::GetInstance()->Drive(moveSpeed * speedMultiplier, turn, true);
}

// Make this return true when this Command no longer needs to run execute()
bool CameraTest::IsFinished()
{
//	RobotVideo::GetInstance()->mutex_lock();
//	bool ret = fabs(RobotVideo::GetInstance()->GetTurn()) < 10;
//	RobotVideo::GetInstance()->mutex_unlock();
	return false; //ret;
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
