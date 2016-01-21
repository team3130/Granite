/*
 * Video.h
 *
 *  Created on: Jan 11, 2016
 *      Author: Mikhail Kyraha
 */

#ifndef SRC_VIDEO_H_
#define SRC_VIDEO_H_

#include <pthread.h>
#include <unistd.h>

class RobotVideo {
private:
	const int CAPTURE_COLS=640, CAPTURE_ROWS=480;
	const int CAPTURE_PORT=0;
	const double CAPTURE_FPS=15;
	const int MIN_AREA=270; // Min area in pixels, 3*(25+40+25) is a rough estimate

	static RobotVideo* m_pInstance;
	pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_t m_thread;
	bool m_connected, m_idle, m_haveLocation, m_haveHeading;
	float m_Ro, m_turn;
	size_t m_sizeLocation, m_sizeHeading;

	// Hide constructors because singleton
	RobotVideo();
	RobotVideo(RobotVideo const&);
	RobotVideo &operator =(RobotVideo const&);
public:
	int m_debug;
	int mutex_lock() { return pthread_mutex_lock(&m_mutex); };
	int mutex_unlock() { return pthread_mutex_unlock(&m_mutex); };
	static RobotVideo* GetInstance()
	{ if(!m_pInstance) m_pInstance = new RobotVideo; return m_pInstance; };
	void Spawn();

	void SetLocationQueueSize(size_t s) {mutex_lock(); m_sizeLocation=s; mutex_unlock();};
	void SetHeadingQueueSize(size_t s) {mutex_lock(); m_sizeHeading=s; mutex_unlock();};

	// These guys need mutex locked but user should do that so can wrap them in a bunch
	float GetTurn() {return m_turn;};
	float GetDistance() {return m_Ro;};

	// Reading/writing a bool is atomic, no need in mutex lock
	bool HaveLocation() {return m_haveLocation;};
	bool HaveHeading() {return m_haveHeading;};
	void Enable() {m_idle = false;};
	void Disable() {m_idle = false;};

protected:
	void Run();

	friend void *VideoThread(void *param);
};

void *VideoThread(void *param);

#endif /* SRC_VIDEO_H_ */
