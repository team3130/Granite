#include <Video.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <list>
#include <algorithm>

static const	cv::Vec3i BlobLower(71,  65,  65);
static const	cv::Vec3i BlobUpper(91, 255, 255);
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
	, m_Ro(0)
	, m_turn(0)
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

void RobotVideo::Run()
{
	cv::VideoCapture capture;
	//open the video stream and make sure it's opened
	//We specify desired frame size and fps in constructor
	//Camera must be able to support specified framesize and frames per second
	//or this will set camera to defaults
	int count=1;
	while (!capture.open(0, 640, 480, 7.5))
	//while (!capture.isOpened())
	{
		std::cerr << "Error connecting to camera stream, retrying " << count<< std::endl;
		count++;
		usleep(1000000);
	}

	//After Opening Camera we need to configure the returned image setting
	//all opencv v4l2 camera controls scale from 0.0 to 1.0

	//vcap.set(CV_CAP_PROP_EXPOSURE_AUTO, 1);
	capture.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0);
	capture.set(CV_CAP_PROP_BRIGHTNESS, 0);
	capture.set(CV_CAP_PROP_CONTRAST, 0);


	//set true to indicate we're connected and the thread is working.
	mutex_lock();
	m_connected = true;
	mutex_unlock();

	DataSet locationsX;
	DataSet locationsY;
	DataSet locationsZ;
	DataSet locationsA;

	int nframe = 0;
	while(true) {
		mutex_lock();
		bool idle = m_idle;
		mutex_unlock();

		if(idle) {
			usleep(1000000);
			continue;
		}

		cv::Mat Im;
		cv::Mat hsvIm;
		cv::Mat BlobIm;
		cv::Mat bw;

		capture >> Im;
		if (Im.empty()) {
			std::cerr << " Error reading from camera" << std::endl;
			usleep(5000000);
			continue;
		}
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
				if (cv::contourArea(cont) > 1000) {
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
				//cv::polylines(Im, target, true, cv::Scalar(0, 200, 255),4);
				cv::Vec4f cameralocation = CalculateLocation(target);

				// Store calculations in a queue but use a list instead so we can iterate
				locationsX.push_front(cameralocation[0]);
				locationsY.push_front(cameralocation[1]);
				locationsZ.push_front(cameralocation[2]);
				locationsA.push_front(cameralocation[3]);
				if (locationsX.size()>5) locationsX.pop_back();
				if (locationsY.size()>5) locationsY.pop_back();
				if (locationsZ.size()>5) locationsZ.pop_back();
				if (locationsA.size()>3) locationsA.pop_back();

				// When we collect enough data get the median value for each coordinate
				// Median rather than average because median tolerate noise better
				if (locationsX.size()>2) cameralocation[0] = locationsX.GetMedian();
				if (locationsY.size()>2) cameralocation[1] = locationsY.GetMedian();
				if (locationsZ.size()>2) cameralocation[2] = locationsZ.GetMedian();
				if (locationsA.size()>2) cameralocation[3] = locationsA.GetMedian();

				mutex_lock();
				m_Ro = sqrtf(cameralocation[0] * cameralocation[0] + cameralocation[1] * cameralocation[1] + cameralocation[2] * cameralocation[2]);
				m_turn = Im.cols / 2 - cameralocation[3];
				mutex_unlock();
				//std::ostringstream oss;
				//oss << heading << " " << Ro << std::endl;
				//cv::putText(Im, oss.str(), textOrg, 1, 2, cv::Scalar(0, 200,255), 2);
			}
		}

		nframe++;
		if(nframe%64 == 0) {
			//cv::imwrite("alpha.png", Im);
		}
		//cv::imshow("Image", Im);
		//cv::imshow("Blob", BlobIm);
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

