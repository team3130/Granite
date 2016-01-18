#include "CameraTest.h"
#include <exception>

CameraTest::CameraTest()
: connected(false)
{
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(chassis);
	try {
		if (capture.open(0, 640, 480, 7.5)) {
			connected = true;
			SmartDashboard::PutString("Camera test", " Camera opened");
		}
		else {
			SmartDashboard::PutString("Camera test", " Error opening camera");
		}
	}
	catch(std::exception& e) {
		SmartDashboard::PutString("Camera test", e.what());
	}
	SmartDashboard::PutNumber("Camera exposure", 0.1);
	SmartDashboard::PutNumber("Camera brightness", 1.0);
	SmartDashboard::PutNumber("Camera contrast", 0.0);
}

// Called just before this Command runs the first time
void CameraTest::Initialize()
{
	if(connected) {
		try {
			cv::Mat Im;
			//After Opening Camera we need to configure the returned image setting
			//all opencv v4l2 camera controls scale from 0.0 to 1.0

			//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
			capture.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, SmartDashboard::GetNumber("Camera exposure",0.1));
			capture.set(CV_CAP_PROP_BRIGHTNESS, SmartDashboard::GetNumber("Camera brightness",1.0));
			capture.set(CV_CAP_PROP_CONTRAST, SmartDashboard::GetNumber("Camera contrast",0.0));
			capture.read(Im);
			if (Im.empty()) {
				SmartDashboard::PutString("Camera test", " Error reading from camera");
			}
			else {
				SmartDashboard::PutString("Camera test", " Camera OK");
				cv::imwrite("alpha.png", Im);
			}
		}
		catch(std::exception& e) {
			SmartDashboard::PutString("Camera test", e.what());
		}
	}
}

// Called repeatedly when this Command is scheduled to run
void CameraTest::Execute()
{

}

// Make this return true when this Command no longer needs to run execute()
bool CameraTest::IsFinished()
{
	return true;
}

// Called once after isFinished returns true
void CameraTest::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraTest::Interrupted()
{

}
