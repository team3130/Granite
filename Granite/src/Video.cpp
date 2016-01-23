#include <Video.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <algorithm>
#include <WPILib.h>
#include <Timer.h>

const char* RobotVideo::IMG_FILE_NAME = "/var/volatile/tmp/alpha.png";
const double RobotVideo::CAPTURE_FPS=15;


static const	cv::Vec3i BlobLower(65, 192,  48);
static const	cv::Vec3i BlobUpper(90, 255, 255);
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
pthread_t   RobotVideo::m_thread;

RobotVideo::RobotVideo()
	: m_mutex(PTHREAD_MUTEX_INITIALIZER)
	, m_connected(false)
	, m_idle(true)
	, m_sizeLocation(7)
	, m_sizeHeading(7)
	, m_boxes()
	, m_locations()
	, m_display(0)
{

}

RobotVideo* RobotVideo::GetInstance()
{
	if(!m_pInstance) {
		m_pInstance = new RobotVideo;
		// Recursion hazard!!! VideoThread() also uses this GetInstance()
		// But the second reentry should not come to this point because m_pInstance will be defined.
		pthread_create(&m_thread, NULL, VideoThread, NULL);
	}
	return m_pInstance;
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

size_t RobotVideo::ProcessContours(std::vector<std::vector<cv::Point>> contours) {
	struct Target {
		double rating;
		std::vector<cv::Point> contour;
	};
	std::vector<struct Target> targets;

	for (std::vector<cv::Point> cont : contours)
	{
		// Only process a contour if it is big enough, otherwise it's either too far away or just a noise
		if (cv::contourArea(cont) > MIN_AREA) {
			double similarity = cv::matchShapes(stencil, cont, CV_CONTOURS_MATCH_I3, 1);

			// Less the similarity index closer the contour matches the stencil shape
			// We are interested only in very similar ones
			if (similarity < 2.0) {
				for (std::vector<struct Target>::iterator it = targets.begin(); it != targets.end(); ++it) {
					// Run through all targets we have found so far and find the position where to insert the new one
					if (similarity < it->rating) {
						targets.insert(it, {similarity, cont});
						break;
					}
				}
				// If there are too many targets after the insert pop the last one
				if (targets.size() >= MAX_TARGETS) targets.pop_back();
			}
		}
	}

	std::vector<cv::Vec3f> locations;
	size_t n_locs = 0;
	for (struct Target target : targets) {
		//Extract 4 corner points assuming the blob is a rectanle, more or less horizontal
		std::vector<cv::Point> hull(4);
		hull[0] = cv::Point(10000, 10000);		// North-West
		hull[1] = cv::Point(0, 10000);			// North-East
		hull[2] = cv::Point(0, 0);				// South-East
		hull[3] = cv::Point(10000, 0);			// South-West
		for (cv::Point point : target.contour) {
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

		if(cv::solvePnP(objectPoints, imagePoints, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP)) {
			cv::Rodrigues(rvec, Rmat);
			locations.push_back( -(Rmat.t() * tvec) );
			n_locs++;
		}
		else {
			locations.push_back(cv::Vec3f(0,0,0));
		}
	}
	mutex_lock();
	m_boxes.clear();
	for (struct Target tar : targets) m_boxes.push_back(tar.contour);
	m_locations = locations;
	mutex_unlock();
	return n_locs;
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
	capture.set(CV_CAP_PROP_BRIGHTNESS, 0.12);
	capture.set(CV_CAP_PROP_CONTRAST, 0);


	//set true to indicate we're connected and the thread is working.
	m_connected = true;

	std::vector<DataSet> locationsX(MAX_TARGETS);
	std::vector<DataSet> locationsY(MAX_TARGETS);
	std::vector<DataSet> locationsZ(MAX_TARGETS);
	std::vector<DataSet> locationsA(MAX_TARGETS);

	PurgeBuffer(capture, CAPTURE_FPS);
	while(true) {
		cv::Mat Im;
		cv::Mat hsvIm;
		cv::Mat BlobIm;
		cv::Mat bw;
		Timer timer;

		timer.Reset();
		timer.Stop();

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
		size_t max_locations = m_sizeLocation;
		size_t max_headings = m_sizeHeading;
		int display = m_display;
		mutex_unlock();

		cv::cvtColor(Im, hsvIm, CV_BGR2HSV);
		cv::inRange(hsvIm, BlobLower, BlobUpper, BlobIm);

		//Extract Contours
		BlobIm.convertTo(bw, CV_8UC1);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(bw, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

		if (contours.size() > 0) {

			ProcessContours(contours);

			for (size_t i = 0; i < m_boxes.size(); ++i) {
				std::vector<cv::Point> box = m_boxes[i];
				cv::Vec3f loc = m_locations[i];

				if(max_headings > 0) {
					locationsA[i].push_front(0.5*(box[0].x + box[1].x));
				}

				if(max_locations > 0) {
					locationsX[i].push_front(loc[0]);
					locationsY[i].push_front(loc[1]);
					locationsZ[i].push_front(loc[2]);
				}

				if (display) {
					std::vector<cv::Point> crosshair;
					crosshair.push_back(cv::Point(box[0].x-5, box[0].y-5));
					crosshair.push_back(cv::Point(box[1].x+5, box[1].y-5));
					crosshair.push_back(cv::Point(box[2].x+5, box[2].y+5));
					crosshair.push_back(cv::Point(box[3].x-5, box[3].y+5));
					cv::polylines(Im, crosshair, true, cv::Scalar(260, 0, 255),2);
				}

				while (locationsX[i].size() > max_locations) locationsX[i].pop_back();
				while (locationsY[i].size() > max_locations) locationsY[i].pop_back();
				while (locationsZ[i].size() > max_locations) locationsZ[i].pop_back();
				while (locationsA[i].size() > max_headings) locationsA[i].pop_back();

				if (max_locations>0 && locationsX[i].size()>0 && locationsY[i].size()>0 && locationsZ[i].size()>0) {
					// When we collect enough data get the median value for each coordinate
					// Median is better than average because median tolerate noise better
					loc[0] = locationsX[i].GetMedian();
					loc[1] = locationsY[i].GetMedian();
					loc[2] = locationsZ[i].GetMedian();
				}

				mutex_lock();
				m_locations[i] = loc;
				m_turns[i] = 1.0 - locationsA[i].GetMedian()/(CAPTURE_COLS/2.0);
				mutex_unlock();
			}
		}

		std::ostringstream oss;
		oss << m_turns[0] << " " << max_locations;
		cv::putText(Im, oss.str(), cv::Point(20,CAPTURE_ROWS-40), 1, 2, cv::Scalar(0, 200,255), 2);
		oss.seekp(0);
		oss << m_locations[0];
		cv::putText(Im, oss.str(), cv::Point(20,CAPTURE_ROWS-18), 1, 2, cv::Scalar(0, 200,255), 2);

		if (display == 1) {
			cv::imwrite(IMG_FILE_NAME, Im);
			//if (!m_idle) cv::imwrite("beta.png", BlobIm);
			m_display = 0;
		}
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

