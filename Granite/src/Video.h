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
	static RobotVideo* m_pInstance;
	pthread_mutex_t m_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_t m_thread;
	bool m_connected, m_idle;
	float m_Ro, m_turn;

	// Hide constructors because singleton
	RobotVideo();
	RobotVideo(RobotVideo const&);
	RobotVideo &operator =(RobotVideo const&);
public:
	int mutex_lock() { return pthread_mutex_lock(&m_mutex); };
	int mutex_unlock() { return pthread_mutex_unlock(&m_mutex); };
	static RobotVideo* GetInstance()
	{ if(!m_pInstance) m_pInstance = new RobotVideo; return m_pInstance; };
	void Spawn();
	void Enable() {mutex_lock(); m_idle = false; mutex_unlock();};
	void Disable() {mutex_lock(); m_idle = false; mutex_unlock();};
	float GetTurn() {float f_turn; mutex_lock(); f_turn = m_turn; mutex_unlock(); return f_turn;};
	float GetDistance() {float f_ro; mutex_lock(); f_ro = m_Ro; mutex_unlock(); return f_ro;};

protected:
	void Run();

	friend void *VideoThread(void *param);
};

void *VideoThread(void *param);

#endif /* SRC_VIDEO_H_ */
