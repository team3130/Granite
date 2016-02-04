#include <Video.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <algorithm>
#include <WPILib.h>
#include <Timer.h>

const char* RobotVideo::IMG_FILE_NAME = "/var/volatile/tmp/alpha.png";
const double RobotVideo::CAPTURE_FPS = 10;
const double RobotVideo::CAM_ANGLE = 24.5;

/**
 * \brief Color filter numbers
 *
 * We use a green ring-light and its color has Hue about 70 in OpenCV's range (0-180)
 * We care about anything that falls between 65 and 90 by the hue but the saturation and
 * brightness can be in a quite wide range.
 */
static const	cv::Vec3i BlobLower(65, 192,  10);
static const	cv::Vec3i BlobUpper(90, 255, 255);

/**
 * \brief Stencil is a simplified "contour" that is an "ideal" shape that we're looking for.
 *
 * The stencil consists of 8 points - the 2d corners of the vision target how it would
 * appear in the picture. The origin is in the left-top corner.
 */
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

/** \brief Real FIRSTSTRONGHOLD tower, inches
 *
 * The coordinates of the real target have to be defined related to an origin and
 * in some arbitrary units. We choose inches and the origin to be the center of the window
 * or more precisely the middle of the top of the vision target.
 */
static const std::vector<cv::Point3f> objectPoints = {
		cv::Point3d(-10, 0, 0),
		cv::Point3d(10, 0, 0),
		cv::Point3d(10, 14, 0),
		cv::Point3d(-10, 14, 0) };

/* Microsoft HD3000 camera, 640x480, inches */
static const cv::Matx33d camera_matrix(
		6.8375310267480415e+002, 0., 3.0453360376910450e+002, 0.,
		6.7992230853496733e+002, 2.5487597233224497e+002, 0., 0., 1. );
static const cv::Matx<double, 5, 1> distortion_coefficients(
		1.3027190902059158e-001, -9.7425921957537043e-001, 0., 0.,
		1.7823261169256015e+000 );
/* avg_reprojection_error: 2.2537737785760148e-001 */

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
	, m_turns(MAX_TARGETS)
	, m_display(false)
{

}

/** \brief Get a pointer to the video processor
 *
 * The Video needs to be a parallel process so its cycles won't interfere with the iterations
 * of the Command based robot code. This GetInstance spawns a posix thread as soon as the
 * object is created. And that thread spins out on its own. The communications then happen
 * via the shared variables and a mutex semaphore pointed to by this pointer.
 */
RobotVideo* RobotVideo::GetInstance()
{
	if(!m_pInstance) {
		m_pInstance = new RobotVideo;

		// Recursion hazard!!! VideoThread() also uses this GetInstance()
		// But the second reentry should not come to this point because m_pInstance will be defined.
		int th = pthread_create(&m_thread, NULL, VideoThread, NULL);
		std::cerr << "RobotVideo thread created " << th << " Thread: " << m_thread << std::endl;
	}
	return m_pInstance;
};

/** \brief Get the median value of the list
 *
 *  DataSet is a very thin wrapper around the std::list of floats.
 *  The only extension is this method GetMedian.
 */
float DataSet::GetMedian()
{
	if (size() > 2) {
		// To find the median an ordered array (vector) is needed.
		// Also std::sort will modify it so we need a copy.
		std::vector<float> ord;
		for (float dp : *this) ord.push_back(dp);
		std::sort(ord.begin(), ord.end());
		return ord[ord.size() / 2];
	}
	// Median makes sense only if the set is of 3 or more items. Otherwise return something.
	else if (size() > 0) return *(this->begin());
	else return 0;
}

/** \brief Purge the camera buffer
 *
 * The stream takes a while to start up, and because of it, images from the camera
 * buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
 * run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
 * before we are at the end of the stream, and can allow processing to begin.
 */
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
			//double similarity = cv::matchShapes(stencil, cont, CV_CONTOURS_MATCH_I3, 1);
			double similarity = 500.0 / cv::contourArea(cont);

			// Less the similarity index closer the contour matches the stencil shape
			// We are interested only in very similar ones
			if (similarity < 2.0) {
				if (targets.empty()) {
					// When we just started the first contour is our best candidate
					targets.push_back({similarity, cont});
				}
				else {
					for (std::vector<struct Target>::iterator it = targets.begin(); it != targets.end(); ++it) {
						// Run through all targets we have found so far and find the position where to insert the new one
						if (similarity < it->rating) {
							targets.insert(it, {similarity, cont});
							break;
						}
					}
					// If there are too many targets after the insert pop the last one
					if (targets.size() > MAX_TARGETS) targets.pop_back();
				}
			}
		}
	}

	std::vector<std::vector<cv::Point>> boxes;
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

		boxes.push_back(hull);
	}

	mutex_lock();
	m_locations = locations;
	m_boxes = boxes;
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

		timer.Start();
		mutex_lock();
		size_t max_locations = m_sizeLocation;
		size_t max_headings = m_sizeHeading;
		bool display = m_display;
		mutex_unlock();

		if(m_idle) {
			// Don't do any processing but sleep for a half of the camera's FPS time.
			useconds_t sleeptime = 1000000 / CAPTURE_FPS;
			if (display) {
				std::ostringstream oss;
				oss << "Time: " << timer.Get();
				cv::putText(Im, oss.str(), cv::Point(20,30), 1, 2, cv::Scalar(0, 200,255), 2);
				cv::putText(Im, "Idle", cv::Point(20,CAPTURE_ROWS-40), 1, 2, cv::Scalar(0, 255,100), 2);
				cv::imwrite(IMG_FILE_NAME, Im);
				m_display = false;
				sleeptime = 1000000 / CAPTURE_FPS / 4;
			}
			usleep(sleeptime);
			continue;
		}

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
				float turn = 0.5*(box[0].x + box[1].x);

				if(max_headings > 0) {
					locationsA[i].push_front(turn);
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

				if (max_headings>0 && locationsA[i].size()>0) {
					turn = locationsA[i].GetMedian();
				}

				mutex_lock();
				m_locations[i] = loc;
				m_turns[i] = 1.0 - turn/(CAPTURE_COLS/2.0);
				mutex_unlock();
			}
		}

		if (display) {
			if (m_turns.size() > 0) {
				std::ostringstream oss;
				oss << CAM_ANGLE * m_turns[0] << " " << max_locations;
				cv::putText(Im, oss.str(), cv::Point(20,CAPTURE_ROWS-40), 1, 2, cv::Scalar(0, 200,255), 2);
			}
			else {
				cv::putText(Im, "No target", cv::Point(20,CAPTURE_ROWS-40), 1, 2, cv::Scalar(0, 100,255), 2);
			}
			if (m_locations.size() > 0) {
				std::ostringstream oss;
				oss << m_locations[0];
				cv::putText(Im, oss.str(), cv::Point(20,CAPTURE_ROWS-18), 1, 2, cv::Scalar(0, 200,255), 2);
			}
			else {
				cv::putText(Im, "No location", cv::Point(20,CAPTURE_ROWS-18), 1, 2, cv::Scalar(0, 100,255), 2);
			}

			std::ostringstream oss;
			oss << 1000.0 * timer.Get() << " msec";
			cv::putText(Im, oss.str(), cv::Point(20,30), 1, 2, cv::Scalar(0, 200,255), 2);
			cv::imwrite(IMG_FILE_NAME, Im);
			//if (!m_idle) cv::imwrite("beta.png", BlobIm);
			m_display = false;
		}
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

