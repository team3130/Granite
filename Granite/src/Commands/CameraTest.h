#ifndef CameraTest_H
#define CameraTest_H

#include "opencv2/opencv.hpp"
#include "WPILib.h"

class CameraTest: public Command
{
	bool connected;
	cv::VideoCapture capture;
public:
	CameraTest();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif
