#include "RobotSensors.h"
#include "Subsystems/Chassis.h"
#include "Video.h"

RobotSensors::RobotSensors()
{
	arduino = new SerialPort(115200, SerialPort::kMXP);
	arduino->SetWriteBufferMode(SerialPort::kFlushOnAccess);
	timer = new Timer();
	this->SetRunWhenDisabled(true);
}

RobotSensors::~RobotSensors()
{
	delete arduino;
	delete timer;
}

// Called just before this Command runs the first time
void RobotSensors::Initialize()
{
	timer->Reset();
	timer->Start();
	SmartDashboard::PutString("DB/String 9", "Test 3130");
}

// Called repeatedly when this Command is scheduled to run
void RobotSensors::Execute()
{
	if(timer->Get() > 0.2) {
		timer->Reset();
		timer->Start();

		size_t nTurns = 0;
		double turn = 0;
		RobotVideo::GetInstance()->mutex_lock();
		nTurns = RobotVideo::GetInstance()->HaveHeading();
		if(nTurns > 0) turn = RobotVideo::GetInstance()->GetTurn(0);
		RobotVideo::GetInstance()->mutex_unlock();
		if (nTurns>0) {
			if (fabs(turn)<2.0) {
				double a = timer->Get();
				uint32_t res = arduino->Write("Solid green\n", 12);
				std::ostringstream oss2;
				oss2 << "ON:" << res << " X:" << a << " T:" << timer->Get();
				SmartDashboard::PutString("DB/String 2", oss2.str());
			}
			else {
				SmartDashboard::PutString("DB/String 2", "off target");
			}
		}
		else {
			SmartDashboard::PutString("DB/String 2", "no target");
		}
	}


	std::ostringstream oss0;
	oss0 << "Angle: " << ChassisSubsystem::GetInstance()->GetAngle();
	SmartDashboard::PutString("DB/String 0", oss0.str());

	std::ostringstream oss1;
	oss1 << "Pos: " << ChassisSubsystem::GetInstance()->GetDistance();
	SmartDashboard::PutString("DB/String 1", oss1.str());
}

// Make this return true when this Command no longer needs to run execute()
bool RobotSensors::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void RobotSensors::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RobotSensors::Interrupted()
{

}
