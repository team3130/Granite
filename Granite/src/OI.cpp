#include "OI.h"
#include "Commands/CameraAim.h"
#include "Commands/CameraFeed.h"

OI* OI::m_pInstance = NULL;

OI::OI()
{
	// Process operator interface input here.
	stickL = new Joystick(0);
	stickR = new Joystick(1);
	gamepad = nullptr; //new Joystick(2);
	trigLeft = new JoystickButton(stickL, 1);
	trigLeft->WhileHeld(new CameraAim(CameraAim::kLeft));
	trigRight = new JoystickButton(stickR, 1);
	trigRight->WhileHeld(new CameraAim(CameraAim::kRight));
}

OI* OI::GetInstance()
{
	if(!m_pInstance) m_pInstance = new OI;
	return m_pInstance;
}
