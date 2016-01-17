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
class ChassisSubsystem: public Subsystem
{
private:
	static ChassisSubsystem* m_pInstance;
	RobotDrive* m_drive;

	const int M_FrontLeft = 2;
	const int M_RearLeft = 0;
	const int M_FrontRight = 3;
	const int M_RearRight = 1;


	ChassisSubsystem();
	ChassisSubsystem(ChassisSubsystem const&);
	ChassisSubsystem& operator=(ChassisSubsystem const&);
public:
	static ChassisSubsystem* GetInstance();
	void InitDefaultCommand();
	void Drive(double move, double turn, bool squaredInputs = false);
};

#endif
