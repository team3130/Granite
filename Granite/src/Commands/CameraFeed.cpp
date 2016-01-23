#include "CameraFeed.h"
#include "Video.h"
#include "Vision/VisionAPI.h"

CameraFeed::CameraFeed()
	: image(frcCreateImage(IMAQ_IMAGE_RGB))
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	SetRunWhenDisabled(true);
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
	RobotVideo::GetInstance()->SetLocationQueueSize(10.0 * SmartDashboard::GetNumber("DB/Slider 0",0));
	if(!RobotVideo::GetInstance()->m_display) {
		frcReadImage(image,RobotVideo::IMG_FILE_NAME);
		CameraServer::GetInstance()->SetImage(image);
		//frcDispose(image);
		RobotVideo::GetInstance()->m_display = true;
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
