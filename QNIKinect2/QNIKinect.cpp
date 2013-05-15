#include "QNIKinect.h"

QKinect::QKinect(const char *file, QObject* parent):_kinect(NULL){
	this->_kinect = new NIKinect();

	this->_kinect->init(file);
	start();
}

QKinect::~QKinect(){
	if(this->_kinect)
		this->_kinect->~NIKinect();
}

void QKinect::run(){
	while(true){
		//this->_oni_grabber->waitForNextFrame();
		this->_kinect->update();

		//this->m_lock.lockForWrite();
		//this->_oni_grabber->copyImageTo(image);
		//this->m_lock.unlock();

		//processor.processImage(image);

		emit kinect_image();
	}
}
