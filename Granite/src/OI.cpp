#include "OI.h"
#include "Commands/CameraTest.h"

OI* OI::m_pInstance = NULL;

OI::OI()
{
	// Process operator interface input here.
	stickL = new Joystick(0);
	stickR = new Joystick(1);
	gamepad = new Joystick(2);
	magicButton = new JoystickButton(stickL, 3);
	magicButton->WhileHeld(new CameraTest());
}

OI* OI::GetInstance()
{
	if(!m_pInstance) m_pInstance = new OI;
	return m_pInstance;
}
