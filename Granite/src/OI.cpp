#include "OI.h"
#include "Commands/CameraAim.h"
#include "Commands/CameraFeed.h"

OI* OI::m_pInstance = NULL;

OI::OI()
{
	// Process operator interface input here.
	stickL = new Joystick(0);
	stickR = new Joystick(1);
	gamepad = new Joystick(2);
	trigLeft = new JoystickButton(stickL, 1);
	trigLeft->WhileHeld(new CameraAim(CameraAim::kLeft));
	trigLeft = new JoystickButton(stickR, 1);
	trigLeft->WhileHeld(new CameraAim(CameraAim::kRight));
	//feedButton = new JoystickButton(stickL, 2);
	//feedButton->WhileHeld(new CameraFeed());
}

OI* OI::GetInstance()
{
	if(!m_pInstance) m_pInstance = new OI;
	return m_pInstance;
}
