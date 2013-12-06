/** 
 * @file	NIKinect2Manager.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	October, 2013 
 * @brief	Declaration of the NIKinect2Manager Class.
 */
#ifndef _NI_KINECT_2_MANAGER
#define _NI_KINECT_2_MANAGER

#include "NIKinect2.h"

class __declspec(dllexport)NIKinect2Manager{
	public:
		NIKinect2Manager();
		~NIKinect2Manager();

		//void add_kinect();
		int initialize_all_kinects();

		bool update_all();

		bool update_ni();
		bool update_info();

		void add_kinect(NIKinect2* kinect);

		NIKinect2* get_kinect(int idx);
		std::vector<NIKinect2*>* get_kinects();
		
	private:	
		std::vector<NIKinect2*>* _kinects;
};

#endif//_NI_KINECT_2_MANAGER