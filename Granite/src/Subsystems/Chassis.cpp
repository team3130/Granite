#include "Subsystems/Chassis.h"
#include "Commands/DefaultDrive.h"

ChassisSubsystem* ChassisSubsystem::m_pInstance = NULL;

ChassisSubsystem* ChassisSubsystem::GetInstance()
{
	if(!m_pInstance) m_pInstance = new ChassisSubsystem;
	return m_pInstance;
}

ChassisSubsystem::ChassisSubsystem()
	: PIDSubsystem("Chassis", 0.05, 0.01, 0.15)
	, moveSpeed(0)
	, m_onPID(false)
{
    m_drive = new RobotDrive(M_FrontLeft,M_RearLeft,M_FrontRight,M_RearRight);
	m_drive->SetSafetyEnabled(false);

	m_cEncoderL = new Encoder(4,5);
	m_cEncoderR = new Encoder(6,7);

	GetPIDController()->Disable();
}

void ChassisSubsystem::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	SetDefaultCommand(new DefaultDriveCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void ChassisSubsystem::Drive(double move, double turn, bool quad)
{
	m_drive->ArcadeDrive(move, turn, quad);
}

void ChassisSubsystem::TankDrive(double left, double right, bool quad)
{
	m_drive->TankDrive(left, right, quad);
}

double ChassisSubsystem::GetAngle()
{
	return ( m_cEncoderL->GetDistance() - m_cEncoderR->GetDistance() ) * 180 / (26.5 * M_PI);
	/*
	 *  Angle is 180 degrees times encoder difference over Pi * the distance between the wheels
	 *	Made from geometry and relation between angle fraction and arc fraction with semicircles.
	 *  Negative because our encoders connected backwards
	 */
}

double ChassisSubsystem::ReturnPIDInput()
{
	return GetAngle();
}

void ChassisSubsystem::UsePIDOutput(double bias)
{
	const double speedLimit = 0.65;
	SmartDashboard::PutNumber("Turn PID bias",bias);
	if(bias >  speedLimit) bias = speedLimit;
	if(bias < -speedLimit) bias = -speedLimit;
	double speed_L = moveSpeed-bias;
	double speed_R = moveSpeed+bias;
	m_drive->TankDrive(speed_L, speed_R, false);
}

double ChassisSubsystem::GetDistance()
{
	return ( m_cEncoderL->GetDistance() + m_cEncoderR->GetDistance() ) / -2.0;
}

void ChassisSubsystem::ResetEncoders()
{
	m_cEncoderL->Reset();
	m_cEncoderR->Reset();
	m_cEncoderL->SetDistancePerPulse(6 * M_PI / 128); // 70/468 = 0.15
	m_cEncoderR->SetDistancePerPulse(6 * M_PI / 256); // 70/937 = 0.07
}

void ChassisSubsystem::HoldAngle(double angle)
{
	GetPIDController()->SetSetpoint(GetAngle() + angle);
	GetPIDController()->Enable();
	m_onPID = true;
}
