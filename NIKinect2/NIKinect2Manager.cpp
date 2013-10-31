/** 
 * @file	NIKinect2Manager.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	October, 2013 
 * @brief	Implementation of the NIKinect2Manager Class.
 *
 * @details	Detailed Information.
 */
#include "NIKinect2Manager.h"

NIKinect2Manager::NIKinect2Manager(){
	this->_kinects = new std::vector<NIKinect2*>();
}

NIKinect2Manager::~NIKinect2Manager(){
	for(int i = 0 ; i < this->_kinects->size() ; ++i){
		NIKinect2* kinect = this->_kinects->at(i);

		kinect->~NIKinect2();
	}
	this->_kinects->clear();
}



int NIKinect2Manager::initialize_all_kinects(){
	if(!NIKinect2::_ni2_initialized)
		NIKinect2::ni_initialize();

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	int n_kinects = deviceList.getSize();

	bool result = false;
	for(int i = 0 ; i < n_kinects ; ++i){
		NIKinect2* kinect = new NIKinect2();

		result = kinect->initialize((char*)deviceList[i].getUri());

		if(result){
			kinect->enable_depth_generator();
			kinect->enable_color_generator();
			kinect->enable_user_generator();

			kinect->set_depth_color_registration(true);

			this->_kinects->push_back(kinect);
		}
	}

	return this->_kinects->size();
}

bool NIKinect2Manager::update_all(){
	if(!NIKinect2::ni_update()){
		return false;
	}

	for(int i = 0 ; i < this->_kinects->size() ; ++i){
		this->_kinects->at(i)->update();
	}

	return true;
}

NIKinect2* NIKinect2Manager::get_kinect(int idx){
	if(idx < this->_kinects->size() && idx >= 0){
		return this->_kinects->at(idx);
	}
	return NULL;
}
		
std::vector<NIKinect2*>* NIKinect2Manager::get_kinects(){
	return this->_kinects;
}