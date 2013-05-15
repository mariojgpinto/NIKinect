#ifndef _Q_NI_KINCT
#define _Q_NI_KINCT

#include <qthread.h>
#include <NIKinect.h>

class __declspec(dllexport) QKinect : public QThread
{
	Q_OBJECT

	public:
		QKinect(const char *file = 0,QObject* parent =0);
        ~QKinect();

    protected:
        void run();

    private:
		NIKinect* _kinect;
		
		//mutable RecursiveQReadWriteLock m_lock;

signals:
		 void kinect_image();
};

#endif //_Q_NI_KINCT