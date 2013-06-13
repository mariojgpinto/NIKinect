#include "QNIKinect.h"

QNIKinect::QNIKinect(const char *file, QObject* parent):_kinect(NULL){
	this->_kinect = new NIKinect();

	this->_kinect->init(file);

	start();
}

QNIKinect::~QNIKinect(){
	if(this->_kinect)
		this->_kinect->~NIKinect();
}

void QNIKinect::run(){
	while(true){
		//this->_oni_grabber->waitForNextFrame();
		this->m_lock.lockForWrite();
		this->_kinect->update();
		this->m_lock.unlock();

		emit kinect_image();
	}
}