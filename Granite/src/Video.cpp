#include <Video.h>

//using namespace cv;

RobotVideo* RobotVideo::m_pInstance = NULL;

void VideoThread(void *param)
{
	RobotVideo *p = RobotVideo::GetInstance();
	p->Run();
}

void RobotVideo::Run()
{
	cv::VideoCapture vcap;
	//open the video stream and make sure it's opened
	//We specify desired frame size and fps in constructor
	//Camera must be able to support specified framesize and frames per second
	//or this will set camera to defaults
	int count=1;
	while (!vcap.open(0, 640, 480, 7.5))
	{
		std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
		count++;
		usleep(1000000);
	}

	//After Opening Camera we need to configure the returned image setting
	//all opencv v4l2 camera controls scale from 0.0 to 1.0

	//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
	vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
	vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
	vcap.set(CV_CAP_PROP_CONTRAST, 0);


	//set true to indicate we're connected and the thread is working.
	mutex_lock();
	m_connected = true;
	mutex_unlock();

	while (true)
	{
		vcap.read(frame);

		// Do stuff

		usleep(1000); //sleep for 1ms
	}

}

RobotVideo::RobotVideo()
	: m_mutex(PTHREAD_MUTEX_INITIALIZER)
	, m_connected(false)
{

}
