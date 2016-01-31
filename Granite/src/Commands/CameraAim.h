#ifndef CameraAim_H
#define CameraAim_H

#include "WPILib.h"

class CameraAim: public Command
{
public:
	Timer timer;
	double m_tolerance;
	CameraAim();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
