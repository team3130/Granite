#include "CameraFeed.h"
#include "Video.h"
#include "Vision/VisionAPI.h"

CameraFeed::CameraFeed()
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
}

// Called just before this Command runs the first time
void CameraFeed::Initialize()
{
	RobotVideo::GetInstance()->SetHeadingQueueSize(0);
	RobotVideo::GetInstance()->SetLocationQueueSize(0);
	RobotVideo::GetInstance()->Enable();
}

// Called repeatedly when this Command is scheduled to run
void CameraFeed::Execute()
{
	if(RobotVideo::GetInstance()->m_debug) {
		Image *image = frcCreateImage(IMAQ_IMAGE_RGB);
		frcReadImage(image,"alpha.png");
		CameraServer::GetInstance()->SetImage(image);
		frcDispose(image);
	}
	else {
		RobotVideo::GetInstance()->m_debug = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool CameraFeed::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void CameraFeed::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraFeed::Interrupted()
{

}
