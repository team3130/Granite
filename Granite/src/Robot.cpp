#include <Subsystems/Chassis.h>
#include <Commands/CameraFeed.h>

class Robot: public IterativeRobot
{
private:
	Command *autonomousCommand;
	LiveWindow *lw;
	CameraFeed *cameraFeed;

	void RobotInit()
	{
		// Create a single static instance of all of your subsystems. The following
		// line should be repeated for each subsystem in the project.
		ChassisSubsystem::GetInstance();
		autonomousCommand = NULL;
		lw = LiveWindow::GetInstance();
		cameraFeed = new CameraFeed();
	}
	
	void DisabledInit()
	{
		// printf("Default %s() method... Overload me!\n", __FUNCTION__);
		//RobotVideo::GetInstance()->Disable();
		cameraFeed->Start();
	}

	void DisabledPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void AutonomousInit()
	{
		cameraFeed->Cancel();
		if (autonomousCommand != NULL)
			autonomousCommand->Start();
	}

	void AutonomousPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void TeleopInit()
	{
		cameraFeed->Cancel();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to 
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != NULL)
			autonomousCommand->Cancel();
	}

	void TeleopPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);

