#ifndef QNIKINECT_H
#define QNIKINECT_H

#include "qnikinect_global.h"

#include <qthread.h>
#include <qreadwritelock.h>
#include <NIKinect.h>

class QNIKINECT_EXPORT QNIKinect : public QThread
//class __declspec(dllexport) QKinect : public QThread
{
	Q_OBJECT

	public:
		QNIKinect(const char *file = 0,QObject* parent =0);
        ~QNIKinect();

        /*! Acquire a lock to ensure data do not get modified by the grabber thread. */
        void acquire_read_lock() { m_lock.lockForRead(); }

        /*! Release the acquired lock. */
        void release_read_lock() { m_lock.unlock(); }


		NIKinect* get_kinect(){return this->_kinect;}

    protected:
        void run();

    private:
		NIKinect* _kinect;
		
		mutable QReadWriteLock m_lock;

signals:
		 void kinect_image();
};

#endif // QNIKINECT_H
