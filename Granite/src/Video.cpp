#include <Video.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <list>
#include <algorithm>
#include <WPILib.h>
#include <Timer.h>

static const	cv::Vec3i BlobLower(66,  65,  65);
static const	cv::Vec3i BlobUpper(96, 255, 255);
static const 	std::vector<cv::Point> stencil = {
		{ 32, 0 },
		{ 26, 76 },
		{ 184, 76 },
		{ 180, 0 },
		{ 203, 0 },
		{ 212, 100 },
		{ 0, 100 },
		{ 9, 0 }
	};

/* Real FIRSTSTRONGHOLD tower, inches */
static const std::vector<cv::Point3f> objectPoints = {
		cv::Point3d(-10, 0, 0),
		cv::Point3d(10, 0, 0),
		cv::Point3d(10, 14, 0),
		cv::Point3d(-10, 14, 0) };

/* Whiteboard res target, millimeters
objectPoints.push_back(cv::Point3d(-135, 0, 0));
objectPoints.push_back(cv::Point3d(135, 0, 0));
objectPoints.push_back(cv::Point3d(135, 150, 0));
objectPoints.push_back(cv::Point3d(-135, 150, 0));
*/

/* Microsoft HD3000 camera, inches */
static const cv::Matx33d camera_matrix(
	7.4230925920305481e+002, 0., 3.0383585011521706e+002, 0.,
	7.4431328863404576e+002, 2.3422929172706634e+002, 0., 0., 1.);
static const cv::Matx<double, 5, 1> distortion_coefficients(
	2.0963551753568421e-001, -1.4796846132520820e+000, 0., 0., 2.7677879392937270e+000);


RobotVideo* RobotVideo::m_pInstance = NULL;

RobotVideo::RobotVideo()
	: m_mutex(PTHREAD_MUTEX_INITIALIZER)
	, m_thread()
	, m_connected(false)
	, m_idle(true)
	, m_haveLocation(false)
	, m_haveHeading(false)
	, m_Ro(0)
	, m_turn(0)
	, m_sizeLocation(7)
	, m_sizeHeading(7)
{

}

void RobotVideo::Spawn()
{
	pthread_create(&m_thread, NULL, VideoThread, NULL);
}

class DataSet : public std::list<float> {
public:
	float GetMedian();
};

float DataSet::GetMedian()
{
	if (size() > 2) {
		std::vector<float> ord;
		for (float dp : *this) ord.push_back(dp);
		std::sort(ord.begin(), ord.end());
		return ord[ord.size() / 2];
	}
	else if (size() > 0) return *(this->begin());
	else return 0;
}

cv::Vec4f CalculateLocation(std::vector<cv::Point> target)
{
	//Extract 4 corner points assuming the blob is a rectanle, more or less horizontal
	std::vector<cv::Point> hull(4);
	hull[0] = cv::Point(10000, 10000);		// North-West
	hull[1] = cv::Point(0, 10000);			// North-East
	hull[2] = cv::Point(0, 0);				// South-East
	hull[3] = cv::Point(10000, 0);			// South-West
	for (cv::Point point : target) {
		if (hull[0].x + hull[0].y > point.x + point.y) hull[0] = point;
		if (hull[1].y - hull[1].x > point.y - point.x) hull[1] = point;
		if (hull[2].x + hull[2].y < point.x + point.y) hull[2] = point;
		if (hull[3].x - hull[3].y > point.x - point.y) hull[3] = point;
	}

	// Make 'em float
	std::vector<cv::Point2f> imagePoints(4);
	imagePoints[0] = hull[0];
	imagePoints[1] = hull[1];
	imagePoints[2] = hull[2];
	imagePoints[3] = hull[3];

	cv::Vec3d rvec, tvec;
	cv::Matx33d Rmat;

	cv::solvePnP(objectPoints, imagePoints, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);
	cv::Rodrigues(rvec, Rmat);

	cv::Vec3f location = -(Rmat.t() * tvec);
	return cv::Vec4f(location[0], location[1], location[2], 0.5*(hull[0].x + hull[1].x));
}

void PurgeBuffer(cv::VideoCapture& vcap, double fps=7.5)
{
	Timer timer;
	double start, end;
	cv::Mat frame;

	timer.Reset();
	timer.Start();
	//run in continuous loop
	while (true)
	{
		start = timer.Get();
		vcap.read(frame);
		end = timer.Get();

		//The stream takes a while to start up, and because of it, images from the camera
		//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
		//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
		//before we are at the end of the stream, and can allow processing to begin.
		if (end - start > 0.5/fps || end >= 5.0)
			break;
	}
}

void RobotVideo::Run()
{
	cv::VideoCapture capture;

	//open the video stream and make sure it's opened
	//We specify desired frame size and fps in constructor
	//Camera must be able to support specified framesize and frames per second
	//or this will set camera to defaults
	int count=1;
	while (!capture.open(CAPTURE_PORT, CAPTURE_COLS, CAPTURE_ROWS, CAPTURE_FPS))
	{
		std::cerr << "Error connecting to camera stream, retrying " << count<< std::endl;
		count++;
		usleep(5 * 1000000);
	}

	//After Opening Camera we need to configure the returned image setting
	//all opencv v4l2 camera controls scale from 0.0 to 1.0
	capture.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0);
	capture.set(CV_CAP_PROP_BRIGHTNESS, 0);
	capture.set(CV_CAP_PROP_CONTRAST, 0);


	//set true to indicate we're connected and the thread is working.
	m_connected = true;

	DataSet locationsX;
	DataSet locationsY;
	DataSet locationsZ;
	DataSet locationsA;
	size_t max_locations, max_headings;

	PurgeBuffer(capture, CAPTURE_FPS);
	while(true) {
		cv::Mat Im;
		cv::Mat hsvIm;
		cv::Mat BlobIm;
		cv::Mat bw;
		cv::Vec4f cameralocation;
		Timer timer;
		bool haveLocation=false, haveHeading=false;

		timer.Reset();
		capture >> Im;
		if (Im.empty()) {
			std::cerr << " Error reading from camera" << std::endl;
			usleep(5 * 1000000);
			continue;
		}

		if(m_idle) {
			// Don't do any processing but sleep for a half of the camera's FPS time.
			SmartDashboard::PutNumber("Video Time", timer.Get());
			usleep(1000000 / CAPTURE_FPS / 2.0);
			continue;
		}

		timer.Start();
		mutex_lock();
		max_locations = m_sizeLocation;
		max_headings = m_sizeHeading;
		mutex_unlock();

		cv::cvtColor(Im, hsvIm, CV_BGR2HSV);
		cv::inRange(hsvIm, BlobLower, BlobUpper, BlobIm);

		//Extract Contours
		BlobIm.convertTo(bw, CV_8UC1);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(bw, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		if (contours.size() > 0) {
			std::vector<cv::Point> target;
			double sim1 = 1000.0;
			for (std::vector<cv::Point> cont : contours)
			{
				// Only process cont if it is big enough, otherwise it's either too far or just a noise
				if (cv::contourArea(cont) > MIN_AREA) {
					double similarity = cv::matchShapes(stencil, cont, CV_CONTOURS_MATCH_I3, 1);
					if (similarity < sim1)
					{
						target = cont;
						sim1 = similarity;
					}
				}
			}

			/*
			for (std::vector<cv::Point> it : contours) {
				cv::polylines(Im, it, true, cv::Scalar(255, 0, 0));
			}
			std::cout << "Similarity " << sim1 << std::endl;
			*/

			if (target.size() > 0 && sim1 < 2.0) {
				haveLocation = true;
				haveHeading = true;
				//cv::polylines(Im, target, true, cv::Scalar(0, 200, 255),4);
				cameralocation = CalculateLocation(target);

				if(max_locations > 0) {
					// Store calculations in a queue but use a list instead so we can iterate
					locationsX.push_front(cameralocation[0]);
					locationsY.push_front(cameralocation[1]);
					locationsZ.push_front(cameralocation[2]);
				}
				if(max_headings > 0) {
					locationsA.push_front(cameralocation[3]);
				}
			}
		}

		float x=0, y=0, z=0, a=0;
		if (locationsX.size() > max_locations) locationsX.pop_back();
		if (locationsY.size() > max_locations) locationsY.pop_back();
		if (locationsZ.size() > max_locations) locationsZ.pop_back();
		if (locationsA.size() > max_headings) locationsA.pop_back();

		if (haveLocation && max_locations == 0) {
			x = cameralocation[0];
			y = cameralocation[1];
			z = cameralocation[2];
		}
		else if (locationsX.size()>0 && locationsY.size()>0 && locationsZ.size()>0) {
			// When we collect enough data get the median value for each coordinate
			// Median is better than average because median tolerate noise better
			x = locationsX.GetMedian();
			y = locationsY.GetMedian();
			z = locationsZ.GetMedian();
			haveLocation = true;
		}
		else haveLocation = false;

		if (haveHeading && max_headings == 0) a = cameralocation[3];
		else if (locationsA.size() > 0) a = locationsA.GetMedian();
		else haveHeading = false;

		mutex_lock();
		if(haveLocation) m_Ro = sqrtf(x*x + y*y + z*z);
		if(haveHeading) m_turn = CAPTURE_COLS/2 - a;
		m_haveHeading = haveHeading;
		m_haveLocation = haveLocation;
		mutex_unlock();

		SmartDashboard::PutNumber("Video Time", timer.Get());
		usleep(1000); //sleep for 1ms
	}
}

void *VideoThread(void *param)
{
	RobotVideo *p = RobotVideo::GetInstance();
	p->Run();
	// The "Run" above should never return
	return NULL;
}

