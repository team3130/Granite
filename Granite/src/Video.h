/*
 * Video.h
 *
 *  Created on: Jan 11, 2016
 *      Author: Mikhail Kyraha
 */

#ifndef SRC_VIDEO_H_
#define SRC_VIDEO_H_

#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <unistd.h>

class RobotVideo {
private:
	static RobotVideo* m_pInstance;
	pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
	bool m_connected;
	cv::Mat frame;

	// Hide constructors because singleton
	RobotVideo();
	RobotVideo(RobotVideo const&);
	RobotVideo &operator =(RobotVideo const&);
public:
	int mutex_lock() { return pthread_mutex_lock(&m_mutex); };
	int mutex_unlock() { return pthread_mutex_unlock(&m_mutex); };
	static RobotVideo* GetInstance()
	{ if(!m_pInstance) m_pInstance = new RobotVideo; return m_pInstance; };
	void Run();
};

void VideoThread(void *param);

#endif /* SRC_VIDEO_H_ */
