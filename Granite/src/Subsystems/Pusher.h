#ifndef Pusher_H
#define Pusher_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class Pusher: public Subsystem
{
private:
	static Pusher* m_pInstance;
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
	Pusher();

public:
	CANTalon* m_motor;

	static Pusher* GetInstance();
	void InitDefaultCommand();
	void Roll(double speed=0);
};

#endif
