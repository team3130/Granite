#include "Pusher.h"

Pusher::Pusher() :
		Subsystem("ExampleSubsystem")
, m_motor(new CANTalon(3))
{

}

Pusher* Pusher::m_pInstance = NULL;
Pusher* Pusher::GetInstance()
{
	if(!m_pInstance) m_pInstance = new Pusher;
	return m_pInstance;
}

void Pusher::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Pusher::Roll(double speed) {
	SmartDashboard::PutNumber("Intake A", m_motor->GetOutputCurrent());
	SmartDashboard::PutNumber("Intake V", m_motor->GetOutputVoltage());
	SmartDashboard::PutNumber("Intake W", m_motor->GetOutputCurrent() * m_motor->GetOutputVoltage());

	m_motor->Set(speed);
}
