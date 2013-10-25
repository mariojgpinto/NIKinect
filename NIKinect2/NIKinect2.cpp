/** 
 * @file	NIKinect2.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	October, 2013 
 * @brief	Implementation of the NIKinect2 Class.
 *
 * @details	Detailed Information.
 */
#include "NIKinect2.h"


bool NIKinect2::_ni2_initialized = false;
int NIKinect2::_ni2_n_streams = 0;
openni::VideoStream** NIKinect2::_ni2_g_streams = new openni::VideoStream*[NIKinect2::_n_generators];
int NIKinect2::_ni2_active_streams = 0;

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	Constructor.
 * @details	Constructor.
 */
NIKinect2::NIKinect2(){
	this->_flag_generators[NIKinect2::NI2_G_DEPTH] = false;
	this->_flag_generators[NIKinect2::NI2_G_COLOR] = false;
	this->_flag_generators[NIKinect2::NI2_G_IR] = false;

	this->_device = new openni::Device();

	this->_g_depth = new openni::VideoStream();
	this->_g_color = new openni::VideoStream();
	this->_g_ir    = new openni::VideoStream();

	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 0] = this->_g_depth;
	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 1] = this->_g_color;
	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 2] = this->_g_ir;
	NIKinect2::_ni2_n_streams += 3;

	this->_frame_depth = new openni::VideoFrameRef();
	this->_frame_color = new openni::VideoFrameRef();
	this->_frame_ir    = new openni::VideoFrameRef();

	this->_nite_user_tracker = new nite::UserTracker();
	this->_nite_user_tracker_frame = new nite::UserTrackerFrameRef();
	//this->_nite_skeleton_states = new nite::SkeletonState();

	this->_last_tick = 0;
	this->_frame_counter = 0;
	this->_frame_rate = 0;

	this->_image_y = 480;
	this->_image_x = 640;
}

/**
 * @brief	Destructor.
 * @details	Destructor.
 */
NIKinect2::~NIKinect2(){
	_nite_user_tracker_frame->release();
	_nite_user_tracker->destroy();
	//nite::NiTE::shutdown();

	this->disable_depth_generator();
	this->disable_color_generator();
	this->disable_ir_generator();
	//openni::OpenNI::shutdown();
}

//-----------------------------------------------------------------------------
// STATIC METHODS
//-----------------------------------------------------------------------------
/**
 * @brief	ni_initialize.
 * @details	ni_initialize.
 */
bool NIKinect2::ni_initialize(){
	if(!NIKinect2::_ni2_initialized){
		openni::Status rc = openni::OpenNI::initialize();

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("ni_initialize","OpenNI Initialization");
		}
		else{
			nite::Status status = nite::NiTE::initialize();

			if(status != nite::STATUS_OK){
				NI2_PRINT_ERROR("ni_initialize","NiTE Initialization");
			}
			else{
				NIKinect2::_ni2_initialized = true;
			}
		}
	}
	return NIKinect2::_ni2_initialized;
}

/**
 * @brief	ni_update.
 * @details	ni_update.
 */
bool NIKinect2::ni_update(){
	if(NIKinect2::_ni2_active_streams){
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(NIKinect2::_ni2_g_streams, 
															 NIKinect2::_ni2_active_streams, 
															 &changedIndex);
		if (rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("ni_update","Error waiting for Streams");
			return false;
		}
		return true;
	}
	return true;
}

/**
 * @brief	ni_shutdown.
 * @details	ni_shutdown.
 */
void NIKinect2::ni_shutdown(){
	nite::NiTE::shutdown();
	
	openni::OpenNI::shutdown();
}

//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------

/**
 * @brief	initialize.
 * @details	initialize.
 */
bool NIKinect2::initialize(char* uri){
	if(!NIKinect2::_ni2_initialized){
		NI2_PRINT_ERROR("initialize","Run NIKinect2::ni_initialize() first.");
		return false;
	}
	const char* deviceURI;

	if(uri){
		deviceURI = uri;
	}
	else{
		deviceURI = openni::ANY_DEVICE;
	}

	openni::Status rc = this->_device->open(deviceURI);

	if(rc != openni::STATUS_OK){
		NI2_PRINT_ERROR("initialize","Error openning device");
		return false;
	}
}

///**
// * @brief	init_variables.
// * @details	init_variables.
// */
//void NIKinect2::init_variables(){
//	this->_flag_generators[NIKinect2::NI2_G_DEPTH] = false;
//	this->_flag_generators[NIKinect2::NI2_G_COLOR] = false;
//	this->_flag_generators[NIKinect2::NI2_G_IR] = false;
//
//	this->_device = new openni::Device();
//
//	this->_g_depth = new openni::VideoStream();
//	this->_g_color = new openni::VideoStream();
//	this->_g_ir    = new openni::VideoStream();
//
//	this->_frame_depth = new openni::VideoFrameRef();
//	this->_frame_color = new openni::VideoFrameRef();
//	this->_frame_ir    = new openni::VideoFrameRef();
//
//	this->_nite_user_tracker = new nite::UserTracker();
//	this->_nite_user_tracker_frame = new nite::UserTrackerFrameRef();
//	this->_nite_skeleton_states = new nite::SkeletonState();
//
//	this->_last_tick = 0;
//	this->_frame_counter = 0;
//	this->_frame_rate = 0;
//}

/**
 * @brief	enable_depth_generator.
 * @details	enable_depth_generator.
 */
bool NIKinect2::enable_depth_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]) return true;

	openni::Status rc = this->_g_depth->create(*this->_device, openni::SENSOR_DEPTH);

	if (rc == openni::STATUS_OK)	{
		rc = this->_g_depth->start();

		if (rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("enable_depth_generator","Couldn't start Depth Stream.");
			this->_g_depth->destroy();
		}
		else{
			if (!this->_g_depth->isValid()){
				NI2_PRINT_ERROR("enable_depth_generator","Depth Stream not Valid.");
			}
			else{
				NIKinect2::_ni2_active_streams++;
				this->_flag_generators[NIKinect2::NI2_G_DEPTH] = true;
			}
		}
	}
	else{
		NI2_PRINT_ERROR("enable_depth_generator","Couldn't find Depth Stream.");
	}

	return this->_flag_generators[NIKinect2::NI2_G_DEPTH];
}

/**
 * @brief	enable_color_generator.
 * @details	enable_color_generator.
 */
bool NIKinect2::enable_color_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_COLOR]) return true;

	openni::Status rc = this->_g_color->create(*this->_device, openni::SENSOR_COLOR);

	if (rc == openni::STATUS_OK)	{
		rc = this->_g_color->start();

		if (rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("enable_color_generator","Couldn't start Color Stream.");
			this->_g_color->destroy();
		}
		else{
			if (!this->_g_color->isValid()){
				NI2_PRINT_ERROR("enable_color_generator","Color Stream not Valid.");
			}
			else{
				NIKinect2::_ni2_active_streams++;
				this->_flag_generators[NIKinect2::NI2_G_COLOR] = true;
			}
		}
	}
	else{
		NI2_PRINT_ERROR("enable_color_generator","Couldn't find Color Stream.");
	}

	return this->_flag_generators[NIKinect2::NI2_G_COLOR];
}

/**
 * @brief	enable_ir_generator.
 * @details	enable_ir_generator.
 */
bool NIKinect2::enable_ir_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_IR]) return true;

	openni::Status rc = this->_g_color->create(*this->_device, openni::SENSOR_IR);

	if (rc == openni::STATUS_OK)	{
		rc = this->_g_ir->start();

		if (rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("enable_ir_generator","Couldn't start IR Stream.");
			this->_g_ir->destroy();
		}
		else{
			if (!this->_g_ir->isValid()){
				NI2_PRINT_ERROR("enable_ir_generator","IR Stream not Valid.");
			}
			else{
				NIKinect2::_ni2_active_streams++;
				this->_flag_generators[NIKinect2::NI2_G_IR] = true;
			}
		}
	}
	else{
		NI2_PRINT_ERROR("enable_ir_generator","Couldn't find IR Stream.");
	}

	return this->_flag_generators[NIKinect2::NI2_G_IR];
}

/**
 * @brief	disable_depth_generator.
 * @details	disable_depth_generator.
 */
bool NIKinect2::disable_depth_generator(){
	if(!this->_flag_generators[NIKinect2::NI2_G_DEPTH]) 
		return true;

	
	this->_g_depth->stop();
	this->_frame_depth->release();
	this->_g_depth->destroy();

	this->_flag_generators[NIKinect2::NI2_G_DEPTH] = false;

	return true;
}

/**
 * @brief	disable_color_generator.
 * @details	disable_color_generator.
 */
bool NIKinect2::disable_color_generator(){
	if(!this->_flag_generators[NIKinect2::NI2_G_COLOR]) 
		return true;

	
	this->_g_color->stop();
	this->_frame_color->release();
	this->_g_color->destroy();

	this->_flag_generators[NIKinect2::NI2_G_COLOR] = false;

	return true;
}

/**
 * @brief	disable_ir_generator.
 * @details	disable_ir_generator.
 */
bool NIKinect2::disable_ir_generator(){
	if(!this->_flag_generators[NIKinect2::NI2_G_IR]) 
		return true;
		
	this->_g_ir->stop();
	this->_frame_ir->release();
	this->_g_ir->destroy();

	this->_flag_generators[NIKinect2::NI2_G_IR] = false;

	return true;
}



//-----------------------------------------------------------------------------
// UPDATE
//-----------------------------------------------------------------------------
/**
 * @brief	update.
 * @details	update.
 */
bool NIKinect2::update(){
	bool result = update_images();

	++this->_frame_counter;
	if (this->_frame_counter == 15)
	{
		double current_tick = cv::getTickCount();
		this->_frame_rate = this->_frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
		this->_last_tick = current_tick;
		this->_frame_counter = 0;
	}

	return result;
}

/**
 * @brief	update_images.
 * @details	update_images.
 */
bool NIKinect2::update_images(){
	openni::Status rc = openni::STATUS_OK;

	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
		rc = this->_g_depth->readFrame(this->_frame_depth);

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("update_images","Error retrieving depth image.");
			return false;
		}

		cv::Mat depthMat16UC1(this->_image_y,this->_image_x,CV_16UC1,(void*)this->_frame_depth->getData());
		depthMat16UC1.copyTo(this->_depth_mat_16);
		depthMat16UC1.convertTo(this->_depth_mat_8, CV_8UC1,0.05);

		cv::inRange(this->_depth_mat_16,1,10000,this->_mask_mat);
	}

	if(this->_flag_generators[NIKinect2::NI2_G_COLOR]){
		rc = this->_g_color->readFrame(this->_frame_color);

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("update_images","Error retrieving color image.");
			return false;
		}

		cv::Mat color_temp(this->_image_y,this->_image_x,CV_8UC3,(void*)this->_frame_color->getData());
		cv::cvtColor(color_temp,this->_color_mat,CV_RGB2BGR);
	}

	//if(this->_flag_generators[NIKinect2::NI2_G_IR]){
	//	rc = this->_g_ir->readFrame(this->_frame_ir);

	//	if(rc != openni::STATUS_OK){
	//		NI2_PRINT_ERROR("update_images","Error retrieving IR image.");
	//	}

	//	cv::Mat color_temp(this->_image_y,this->_image_x,CV_8UC3,(void*)this->_frame_ir->getData());
	//	cv::cvtColor(color_temp,this->_color_mat,CV_RGB2BGR);
	//}
	
	return true;
}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	Gets a copy of the Depth cv::Mat.
 * @details	Gets a copy of the Depth cv::Mat, if the Depth Generator 
 *			(_depth_generator) is active.
 *
 * @param[out]	depth
 *				Copy of the Depth cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_depth_16(cv::Mat &depth){
	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
		this->_depth_mat_16.copyTo(depth);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	Gets a copy of the Depth cv::Mat.
 * @details	Gets a copy of the Depth cv::Mat, if the Depth Generator 
 *			(_depth_generator) is active.
 *
 * @param[out]	depth
 *				Copy of the Depth cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_depth_8(cv::Mat &depth){
	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
		this->_depth_mat_8.copyTo(depth);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	Gets a copy of the Mask cv::Mat.
 * @details	Gets a copy of the Mask cv::Mat, if the Depth Generator 
 *			(_depth_generator) is active.
 *
 * @param[out]	mask
 *				Copy of the Mask cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_mask(cv::Mat &mask){
	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
		this->_mask_mat.copyTo(mask);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	Gets a copy of the Color cv::Mat.
 * @details	Gets a copy of the Color cv::Mat, if the Image Generator 
 *			(_image_generator) is active.
 *
 * @param[out]	color
 *				Copy of the Image cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_color(cv::Mat &color){
	if(this->_flag_generators[NIKinect2::NI2_G_COLOR]){
		this->_color_mat.copyTo(color);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	get_frame_rate.
 * @details	get_frame_rate.
 */
double NIKinect2::get_frame_rate(){
	return this->_frame_rate;
}