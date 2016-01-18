#ifndef CameraTest_H
#define CameraTest_H

#include "WPILib.h"
#include "Video.h"
#include "Subsystems/Chassis.h"

class CameraTest: public Command
{
public:
	CameraTest();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
