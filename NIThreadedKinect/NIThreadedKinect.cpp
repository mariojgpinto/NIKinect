/** 
 * @file	NIThreadedKinect.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	June, 2013 
 * @brief	Implementation of the NIThreadedKinect Class.
 */

#include "NIThreadedKinect.h"

/**
 *
 */
NIThreadedKinect::NIThreadedKinect(){
	this->_flags_threads[CAPTURE_T] = false;
	this->_flags_threads[POINT_CLOUD_T] = false;

	this->_running[CAPTURE_T] = false;
	this->_running[POINT_CLOUD_T] = false;

	this->_mutex[CAPTURE_T] = NULL;
	this->_mutex[POINT_CLOUD_T] = NULL;

	this->_threads[CAPTURE_T] = NULL;
	this->_threads[POINT_CLOUD_T] = NULL;

	this->_point_2d = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);
	this->_point_3d = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);
	this->_point_3d_access = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);
	
	this->_pcl_cloud.width = 640*480;
	this->_pcl_cloud.height = 1;
	this->_pcl_cloud.points.resize (this->_pcl_cloud.width * this->_pcl_cloud.height);

}

/**
 *
 */
NIThreadedKinect::~NIThreadedKinect(){

}


//-----------------------------------------------------------------------------
// CONTROL
//-----------------------------------------------------------------------------

/**
 *
 */
void NIThreadedKinect::start_thread(NIThreadedKinect::THREAD_ID id){
	this->_flags_threads[id] = true;

	//Setup?

	this->_threads[id] = (id == NIThreadedKinect::CAPTURE_T) ? new boost::thread(&NIThreadedKinect::run_kinect,this) : new boost::thread(&NIThreadedKinect::run_point_cloud,this);
	this->_mutex[id] = new boost::mutex();
}

/**
 * @brief	Checks if the thread is running.
 * @details	
 *
 * @retval	true if the thread is running.
 * @retval	false if the thread is not running.
 */
bool NIThreadedKinect::is_running(NIThreadedKinect::THREAD_ID id){
	return 	this->_running[id];
}

/**
 * @brief	Set the _running flag to @c false.
 * @details	If a thread is running. It will stop in the next iteration of the cycle.
 *			@see update_threaded.
 *
 * @todo	Is there need to shutdown something else?
 */
void NIThreadedKinect::stop_thread(NIThreadedKinect::THREAD_ID id){
	this->_running[id] = false;

	//Is it ok to do this?
	this->_threads[id]->join();
}

//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------

/**
 *
 */
void NIThreadedKinect::run_kinect(){
	this->_running[NIThreadedKinect::CAPTURE_T] = true;

	XnStatus rc;
	bool result = false;

	while(this->_running[NIThreadedKinect::CAPTURE_T]){
			
		rc = this->_context.WaitAnyUpdateAll();
		if (rc != XN_STATUS_OK){
			printf("Read failed: %s\n", xnGetStatusString(rc));
			return;
		}

		this->_mutex[NIThreadedKinect::CAPTURE_T]->lock();
		result = this->update_kinect();
		this->_mutex[NIThreadedKinect::CAPTURE_T]->unlock();

		if(!result)
			//TODO How to destroy the thread?
			return;
	}
}

/**
 *
 */
void NIThreadedKinect::run_point_cloud(){
	this->_running[NIThreadedKinect::POINT_CLOUD_T] = true;

	bool result = false;

	while(this->_running[NIThreadedKinect::POINT_CLOUD_T]){
		//Lock the Capture Mutex to copy data
		this->_mutex[NIThreadedKinect::CAPTURE_T]->lock();
		this->_depth_md_copy.CopyFrom(this->_depth_md);
		this->_color_mat.copyTo(_color_mat_copy);
		this->_mutex[NIThreadedKinect::CAPTURE_T]->unlock();

		result = this->update_point_cloud();

		if(!result)
			//TODO How to destroy the thread?
			return;
		else{
			new_point_cloud = true;
		}
	}
}

/**
 * @brief	Updates the generator's information.
 * @details	Used in threaded applications. Used in the \sa run method.
 *
 * @TODO	Implement Other Generators and SceneAnalyzer update.
 */
bool NIThreadedKinect::update_kinect(){
	//Updates Depth Variables
	if(this->_flags[NIKinect::DEPTH_G]){
		this->_depth_generator.GetMetaData(this->_depth_md);

		if(this->_flags_processing[NIKinect::DEPTH_P]){
			cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) this->_depth_md.Data());
			depthMat16UC1.copyTo(this->_depth_mat);
		}

		if(this->_flags_processing[NIKinect::DEPTH_P]){
			//cv::threshold src uses 8-bit or 32-bit floating point so you have to convert before use. Loss of performance.
			//depthMat16UC1.convertTo(mask8,CV_8UC1);
			//cv::threshold(mask8,this->_mask_mat,0.1,255,CV_THRESH_BINARY);
			cv::inRange(this->_depth_mat,1,10000,this->_mask_mat);
		}

		if(this->_flags_processing[NIKinect::DEPTH_COLOR]){
			double min = this->_min_depth;
			double max = this->_max_depth;
			NIKinect::compute_color_encoded_depth(this->_depth_mat,this->_depth_as_color_mat,&min,&max);
		}
	}

	//Updates Image Variables
	if(this->_flags[NIKinect::IMAGE_G]){
		this->_image_generator.GetMetaData(_image_md);

		if(this->_flags_processing[NIKinect::IMAGE_P]){
			cv::Mat color_temp(480,640,CV_8UC3,(void*) _image_md.Data());
			cv::cvtColor(color_temp,_color_mat,CV_RGB2BGR);
		}
	}

	this->update_frame_rate();

	return true;
}

/**
 *
 */
bool NIThreadedKinect::update_point_cloud(){
	int ac = 0;
	for(int y=0; y<XN_VGA_Y_RES; y+=_point_step) { 
		for(int x=0; x<XN_VGA_X_RES; x+=_point_step) { 
			XnPoint3D point1;
			point1.X = x; 
			point1.Y = y; 
			point1.Z = this->_depth_md_copy[y * XN_VGA_X_RES + x]; 

			//this->_point_2d[y * XN_VGA_X_RES + x] = point1;
			this->_point_2d[ac++] = point1;
		}
	} 

	this->_n_points = (XN_VGA_Y_RES*XN_VGA_X_RES) / (float)(_point_step*_point_step);

	_depth_generator.ConvertProjectiveToRealWorld(this->_n_points, this->_point_2d, this->_point_3d);

	if(this->_flags_processing[NIKinect::POINT_CLOUD_PCL]){
		this->_pcl_cloud.clear();
		for(int i = 0 ; i < this->_n_points ; ++i){
			this->_pcl_cloud.push_back(pcl::PointXYZ(this->_point_3d[i].X,this->_point_3d[i].Y,this->_point_3d[i].Z));
		}

		this->_mutex[NIThreadedKinect::POINT_CLOUD_T]->lock();
			_pcl_cloud_access = _pcl_cloud;
		this->_mutex[NIThreadedKinect::POINT_CLOUD_T]->unlock();
	}
	else{
		this->_mutex[NIThreadedKinect::POINT_CLOUD_T]->lock();
		memcpy(this->_point_3d_access,this->_point_3d,sizeof(XnPoint3D) * this->_n_points);
		this->_mutex[NIThreadedKinect::POINT_CLOUD_T]->unlock();
	}

	return true;
}



//-----------------------------------------------------------------------------
// ACCESS POINT CLOUDS
//-----------------------------------------------------------------------------

/** 
 * @brief	
 * @details	
 * 
 * @return	
 *			
 */
XnPoint3D** NIThreadedKinect::get_3d_points(){
	return &this->_point_3d_access;
}

/** 
 * @brief	
 * @details	
 * 
 * @return	
 *			
 */
pcl::PointCloud<pcl::PointXYZ> * NIThreadedKinect::get_point_cloud(){
	if(this->_flags_processing[NIKinect::POINT_CLOUD_PCL] && new_point_cloud){
		this->new_point_cloud = false;
		return &this->_pcl_cloud_access;
	}
	else
		return NULL;
}

/** 
 * @brief	
 * @details	
 *
 * @param[out]		point_3d
 *					
 *
 * @retval	true	
 * @retval	false	
 */
bool NIThreadedKinect::copy_3d_points(XnPoint3D *point_3d){
	if(new_point_cloud){
		memcpy(point_3d,_point_3d,sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES);
		new_point_cloud = false;
		return true;
	}
	else
		return false;
}

/** 
 * @brief	
 * @details	
 *
 * @param[out]		cloud
 *					
 *
 * @retval	true	
 * @retval	false	
 */
bool NIThreadedKinect::copy_point_cloud(pcl::PointCloud<pcl::PointXYZ> &cloud){
	if(new_point_cloud){	
		cloud.clear();

		if(this->_flags_processing[NIKinect::POINT_CLOUD_PCL]){
			cloud = this->_pcl_cloud_access;
		}
		else{
			for(int i = 0 ; i < this->_n_points ; ++i){
				if(this->_point_3d_access[i].Z > 0)
					cloud.push_back(pcl::PointXYZ(this->_point_3d_access[i].X,this->_point_3d_access[i].Y,this->_point_3d_access[i].Z));
			}
		}
		this->new_point_cloud = false;

		return true;
	}
	else
		return false;
}

//-----------------------------------------------------------------------------
// ACCESS - MUTEX
//-----------------------------------------------------------------------------

/**
 * @brief	Lock the NIKinect's mutex (_mutex).
 * @details	If it is already locked, it block the execution.
 */
void NIThreadedKinect::mutex_lock(NIThreadedKinect::THREAD_ID id){
	this->_mutex[id]->lock();
}

/**
 * @brief	Try to lock the NIKinect's mutex (_mutex).
 * @details	If it is already locked, it does not block the execution and returns.
 *
 * @retval	true if the lock is acquired.
 * @retval	false if the lock was already locked.
 */
bool NIThreadedKinect::mutex_try_lock(NIThreadedKinect::THREAD_ID id){
	return this->_mutex[id]->try_lock();
}

/**
 * @brief	
 * @details	
 *
 */
void NIThreadedKinect::mutex_unlock(NIThreadedKinect::THREAD_ID id){
	this->_mutex[id]->unlock();
}