#ifndef CHASSIS_H
#define CHASSIS_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

/** Robot's main chassis, or drivetrain.
 *
 * This must be a singleton class because a robot usually can have only one chassis.
 * The class manages all the driving motors and all methods of driving itself.
 * All activities with the drivetrain must be done via its public methods.
 */
class ChassisSubsystem: public PIDSubsystem
{
private:
	static ChassisSubsystem* m_pInstance;
	RobotDrive* m_drive;

	const int M_FrontLeft = 2;
	const int M_RearLeft = 0;
	const int M_FrontRight = 3;
	const int M_RearRight = 1;
	Encoder* m_cEncoderL;
	Encoder* m_cEncoderR;
	double moveSpeed;
	bool m_onPID;

	ChassisSubsystem();
	ChassisSubsystem(ChassisSubsystem const&);
	ChassisSubsystem& operator=(ChassisSubsystem const&);
public:
	static ChassisSubsystem* GetInstance();
	void InitDefaultCommand();
	virtual double ReturnPIDInput();
	virtual void UsePIDOutput(double outputAngle);
	double GetDistance();
	void ResetEncoders();
	double GetAngle();
	void HoldAngle(double angle = 0);
	void ReleaseAngle() { GetPIDController()->Disable(); m_onPID=false; };

	void Drive(double move, double turn, bool squaredInputs = false);
	void TankDrive(double left, double right, bool squaredInputs = false);
};

#endif
