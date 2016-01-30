#include <Commands/DefaultDrive.h>
#include <Subsystems/Chassis.h>
#include <OI.h>
#include <Video.h>

/// Default constructor of the class.
DefaultDriveCommand::DefaultDriveCommand()
{
	Requires(ChassisSubsystem::GetInstance());
}

/// Called just before this Command runs the first time.
void DefaultDriveCommand::Initialize()
{
	ChassisSubsystem::GetInstance()->ResetEncoders();
	ChassisSubsystem::GetInstance()->ReleaseAngle();
	ChassisSubsystem::GetInstance()->Drive(0,0);
}

/// Called repeatedly when this Command is scheduled to run.
void DefaultDriveCommand::Execute()
{
	OI* oi = OI::GetInstance();
	if(oi) {
		// Y-axis positive is down. We want positive - up. Flip it!
		double LSpeed = -oi->stickL->GetY();
		double RSpeed = -oi->stickR->GetY();
		double LMultiplier = (0.5 * oi->stickL->GetZ()) + 0.5;
		double RMultiplier = (0.5 * oi->stickR->GetZ()) + 0.5;

		// Only driving manual should require Quadratic inputs. By default it should be turned off
		// Therefore here we turn it on explicitly.
		ChassisSubsystem::GetInstance()->TankDrive(LSpeed * LMultiplier, RSpeed * RMultiplier, true);
	}
}

/// Make this return true when this Command no longer needs to run execute().
/// \return always false since this is the default command and should never finish.
bool DefaultDriveCommand::IsFinished()
{
	return false;
}

/// Called once after isFinished returns true
void DefaultDriveCommand::End()
{
}

/// Called when another command which requires one or more of the same
/// subsystems is scheduled to run
void DefaultDriveCommand::Interrupted()
{
}
