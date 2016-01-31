#ifndef CameraAim_H
#define CameraAim_H

#include "WPILib.h"

class CameraAim: public Command
{
public:
	enum Target_side {
		kLeft,
		kRight
	};
	Timer timer;
	double m_tolerance;
	bool m_side;
	CameraAim(Target_side side=kLeft);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
