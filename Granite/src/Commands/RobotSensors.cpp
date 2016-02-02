#include "RobotSensors.h"
#include "Subsystems/Chassis.h"
#include "Video.h"

RobotSensors::RobotSensors()
{
	arduino = new SerialPort(9600, SerialPort::kMXP /*, SerialPort::kUSB*/);
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
	if(timer->Get() > 1.0) {
		timer->Reset();
		timer->Start();
/*
		if( chassis->m_cEncoderL->GetRate() > 6 ) arduino->Write("3", 1);
		else if( chassis->m_cEncoderL->GetRate() < -6 ) arduino->Write("4", 1);
		else arduino->Write("5", 1);

		if( lifter->GetSpeed() > 0.5 ) arduino->Write("6", 1);
		else if(lifter->GetSpeed() < -0.5) arduino->Write("7", 1);
		else arduino->Write("8", 1);

		if( CommandBase::pusher->GetDir() > 0) arduino->Write("9", 1);
		else if( CommandBase::pusher->GetDir() < 0) arduino->Write("10", 1);
		else arduino->Write("11", 1);

		if(!lifterZero) {
			arduino->Write("Z", 1);
		} */
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
