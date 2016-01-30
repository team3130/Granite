#include "RobotSensors.h"

RobotSensors::RobotSensors()
{
	arduino = new SerialPort(9600, SerialPort::kUSB);
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
	SmartDashboard::PutNumber("SDB Test", 3130);
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
		}

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
