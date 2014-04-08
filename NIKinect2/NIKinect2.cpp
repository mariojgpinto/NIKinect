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
	this->xx[0] = 0;
	this->yy[0] = 0;
	this->xx[1] = 0;
	this->yy[1] = 0;
	this->_flag_generators[NIKinect2::NI2_G_DEPTH] = false;
	this->_flag_generators[NIKinect2::NI2_G_COLOR] = false;
	this->_flag_generators[NIKinect2::NI2_G_IR] = false;
	this->_flag_generators[NIKinect2::NI2_G_USER] = false;
	this->_flag_generators[NIKinect2::NI2_G_GESTURES] = false;

	this->_device = new openni::Device();

	this->_g_depth = new openni::VideoStream();
	this->_g_color = new openni::VideoStream();
	this->_g_ir    = new openni::VideoStream();

	//NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 0] = this->_g_color;
	//NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 1] = this->_g_depth;
	//NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 2] = this->_g_ir;
	//NIKinect2::_ni2_n_streams += 3;

	this->_frame_depth = new openni::VideoFrameRef();
	this->_frame_color = new openni::VideoFrameRef();
	this->_frame_ir    = new openni::VideoFrameRef();

	this->_nite_user_tracker = new nite::UserTracker();
	this->_nite_user_tracker_frame = new nite::UserTrackerFrameRef();
	this->_nite_skeleton_states = (nite::SkeletonState*)malloc(sizeof(nite::SkeletonState) * _max_users);//new nite::SkeletonState();

	for(int i = 0 ; i < NIKinect2::_max_users ; ++i){
		this->_nite_skeleton_states[i] = nite::SKELETON_NONE;
	}

	this->_users = new std::vector<nite::UserData*>();
	this->_users_ids = new std::vector<int>();

	this->_nite_hand_tracker = new nite::HandTracker();
	this->_nite_hand_tracker_frame = new nite::HandTrackerFrameRef();

	this->_last_tick = 0;
	this->_frame_counter = 0;
	this->_frame_rate = 0;

	this->_mode_color.setFps(30);
	this->_mode_color.setPixelFormat(openni::PixelFormat(ONI_PIXEL_FORMAT_RGB888));
	this->_mode_color.setResolution(640,480);

	this->_mode_depth.setFps(30);
	this->_mode_depth.setPixelFormat(openni::PixelFormat(ONI_PIXEL_FORMAT_DEPTH_1_MM));
	this->_mode_depth.setResolution(640,480);

	this->_flag_plane = false;
	this->_floor_plane = new nite::Plane();
}

/**
 * @brief	Destructor.
 * @details	Destructor.
 */
NIKinect2::~NIKinect2(){
	this->disable_user_generator();
	//nite::NiTE::shutdown();
	this->disable_ir_generator();
	this->disable_depth_generator();
	this->disable_color_generator();
	
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
	for(int i = 0 ; i < NIKinect2::_ni2_n_streams ; ++i){
		if(NIKinect2::_ni2_g_streams[i]){
			NIKinect2::_ni2_g_streams[i]->stop();
			NIKinect2::_ni2_g_streams[i]->destroy();
		}
	}
	NIKinect2::_ni2_n_streams = 0;
	NIKinect2::_ni2_active_streams = 0;
	NIKinect2::_ni2_initialized = false;

	nite::NiTE::shutdown();
	
	openni::OpenNI::shutdown();
}

/**
 * @brief	NIKinect2::distanceToPlane(float pt_x, float pt_y, float pt_z, float a, float b, float c, float d).
 * @details	NIKinect2::distanceToPlane(float pt_x, float pt_y, float pt_z, float a, float b, float c, float d).
 */
float NIKinect2::distance_to_plane(float pt_x, float pt_y, float pt_z, float a, float b, float c, float d){
	float v = a*pt_x + b*pt_y + c*pt_z + d;
	v /= sqrt(a*a+b*b+c*c);
	return std::abs(v);
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
		NI2_PRINT_ERROR("initialize","Error openning device (%s)", deviceURI);
		return false;
	}

	return true;
}

/**
 * @brief	set_depth_color_registration.
 * @details	set_depth_color_registration.
 */
bool NIKinect2::set_depth_color_registration(bool enable){
	if(!this->_flag_generators[NIKinect2::NI2_G_COLOR] || !this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
		NI2_PRINT_ERROR("set_depth_color_registration","Both Depth and Color Generators must be created before calling this method.");
		return false;
	}

	if(enable){
		if(this->_device->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)){
			openni::Status rc = this->_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

			if(rc != openni::STATUS_OK){
				NI2_PRINT_ERROR("set_depth_color_registration","Error turning ON registration mode on function \"setImageRegistrationMode\".");
				return false;
			}
		}
		else{
			NI2_PRINT_ERROR("set_depth_color_registration","Registration Mode not Supported by the device.");
			return false;
		}
	}
	else{
		openni::Status rc = this->_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("set_depth_color_registration","Error turning OFF registration mode on function \"setImageRegistrationMode\".");
			return false;
		}
	}

	return true;
}

/**
 * @brief	set_color_hd.
 * @details	set_color_hd.
 */
bool NIKinect2::set_color_hd(bool enable){
	if(this->_g_color->getVideoMode().getResolutionX() == 640){
		if(enable){
			this->_mode_color.setFps(12);
			this->_mode_color.setResolution(1280,960);
			this->_mode_color.setPixelFormat(openni::PixelFormat(ONI_PIXEL_FORMAT_RGB888));
		}
		else
			return  true;
	}
	else{
		if(enable)
			return true;
		else{
			this->_mode_color.setFps(30);
			this->_mode_color.setResolution(640,480);
			this->_mode_color.setPixelFormat(openni::PixelFormat(ONI_PIXEL_FORMAT_RGB888));
		}
	}

	if(this->disable_color_generator()){
		return this->enable_color_generator();
	}

	return false;
}

/**
 * @brief	enable_depth_generator.
 * @details	enable_depth_generator.
 */
bool NIKinect2::enable_depth_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]) return true;

	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 0] = this->_g_depth;
	NIKinect2::_ni2_n_streams ++;

	openni::Status rc = this->_g_depth->create(*this->_device, openni::SENSOR_DEPTH);

	if (rc == openni::STATUS_OK)	{
		rc = this->_g_depth->setVideoMode(this->_mode_depth);

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("enable_depth_generator","Couldn't set Video Mode.");
			this->_g_depth->destroy();
		}
		else{
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

	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams + 0] = this->_g_color ;
	NIKinect2::_ni2_n_streams ++;

	openni::Status rc = this->_g_color->create(*this->_device, openni::SENSOR_COLOR);

	if (rc == openni::STATUS_OK){
		rc = this->_g_color->setVideoMode(this->_mode_color);

		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("enable_color_generator","Couldn't set Video Mode.");
			this->_g_color->destroy();
		}
		else{
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
	}
	else{
		NI2_PRINT_ERROR("enable_color_generator","Couldn't find Color Stream.");
	}

	if(this->_flag_generators[NIKinect2::NI2_G_COLOR]){
		rc = this->_g_color->addNewFrameListener(this);
		printf("AddFrameListener\n");
	}

	return this->_flag_generators[NIKinect2::NI2_G_COLOR];
}

/**
 * @brief	enable_ir_generator.
 * @details	enable_ir_generator.
 */
bool NIKinect2::enable_ir_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_IR]) return true;

	NIKinect2::_ni2_g_streams[NIKinect2::_ni2_n_streams] = this->_g_ir;
	NIKinect2::_ni2_n_streams++;

	openni::Status rc = this->_g_ir->create(*this->_device, openni::SENSOR_IR);

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
 * @brief	enable_user_generator.
 * @details	enable_user_generator.
 */
bool NIKinect2::enable_user_generator(){
	if(this->_flag_generators[NIKinect2::NI2_G_USER]) return true;

	nite::Status status = nite::STATUS_OK;

	status = this->_nite_user_tracker->create(this->_device);

	if(status != nite::STATUS_OK){
		NI2_PRINT_ERROR("enable_user_generator","Couldn't create UserTracker.");
	}
	else{
		this->_flag_generators[NIKinect2::NI2_G_USER] = true;
	}

	return this->_flag_generators[NIKinect2::NI2_G_USER];
}

/**
 * @brief	enable_hand_tracker.
 * @details	enable_hand_tracker.
 */
bool NIKinect2::enable_hand_tracker(){
	if(this->_flag_generators[NIKinect2::NI2_G_GESTURES]) return true;

	nite::Status status = nite::STATUS_OK;

	status = this->_nite_hand_tracker->create(this->_device);

	if(status != nite::STATUS_OK){
		NI2_PRINT_ERROR("enable_hand_tracker","Couldn't create HandTracker.");
	}
	else{
		this->_nite_hand_tracker->startGestureDetection(nite::GESTURE_CLICK);
		this->_nite_hand_tracker->startGestureDetection(nite::GESTURE_WAVE);

		this->_flag_generators[NIKinect2::NI2_G_GESTURES] = true;
	}

	return this->_flag_generators[NIKinect2::NI2_G_GESTURES];
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

/**
 * @brief	disable_user_generator.
 * @details	disable_user_generator.
 */
bool NIKinect2::disable_user_generator(){
	if(!this->_flag_generators[NIKinect2::NI2_G_USER]) 
		return true;

	
	//this->_nite_user_tracker->stopPoseDetection();
	//this->_nite_user_tracker->stopSkeletonTracking();

	this->_nite_user_tracker_frame->release();
	this->_nite_user_tracker->destroy();

	this->_flag_generators[NIKinect2::NI2_G_USER] = false;

	return true;
}

/**
 * @brief	disable_hand_tracker.
 * @details	disable_hand_tracker.
 */
bool NIKinect2::disable_hand_tracker(){
	if(!this->_flag_generators[NIKinect2::NI2_G_GESTURES]) 
		return true;

	
	//this->_nite_user_tracker->stopPoseDetection();
	//this->_nite_user_tracker->stopSkeletonTracking();

	this->_nite_hand_tracker_frame->release();
	this->_nite_hand_tracker->destroy();

	this->_flag_generators[NIKinect2::NI2_G_GESTURES] = false;

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
	bool result = true;
	
	result = update_images();

	if(!result)
		goto end;

	if(this->_flag_generators[NIKinect2::NI2_G_USER]){
		result = update_nite();
	}

	end:
	
	++this->_frame_counter;
	if (this->_frame_counter == 15){
		double current_tick = cv::getTickCount();
		this->_frame_rate = this->_frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
		this->_last_tick = current_tick;
		this->_frame_counter = 0;

		printf("FrameRateNI: %.2f\n",this->_frame_rate);
	}

	return result;
}

/**
 * @brief	update_images.
 * @details	update_images.
 */
bool NIKinect2::update_images(){
	openni::Status rc = openni::STATUS_OK;

	//printf("Up.");
	//int temp;
	//openni::OpenNI::waitForAnyStream(NIKinect2::_ni2_g_streams,NIKinect2::_ni2_active_streams,&temp);
	//printf("..dated");
	//if(this->_flag_generators[NIKinect2::NI2_G_DEPTH]){
	//	//printf(" read ");
	//	rc = this->_g_depth->readFrame(this->_frame_depth);
	//	//printf("depth ");
	//	if(rc != openni::STATUS_OK){
	//		NI2_PRINT_ERROR("update_images","Error retrieving depth image.");
	//		return false;
	//	}

	//	cv::Mat depthMat16UC1(this->_mode_depth.getResolutionY(),this->_mode_depth.getResolutionX(),CV_16UC1,(void*)this->_frame_depth->getData());
	//	depthMat16UC1.copyTo(this->_depth_mat_16);
	//	depthMat16UC1.convertTo(this->_depth_mat_8, CV_8UC1,0.05);

	//	cv::inRange(this->_depth_mat_16,1,10000,this->_mask_mat);
	//}

	////ni_update();
	////openni::OpenNI::waitForAnyStream(NIKinect2::_ni2_g_streams,NIKinect2::_ni2_active_streams,&temp);
	////if(this->_flag_generators[NIKinect2::NI2_G_COLOR]){
	////	//printf(" read ");
	////	rc = this->_g_color->readFrame(this->_frame_color);
	////	//printf(" color ");
	////	if(rc != openni::STATUS_OK){
	////		NI2_PRINT_ERROR("update_images","Error retrieving color image.");
	////		return false;
	////	}

	////	cv::Mat color_temp(this->_mode_color.getResolutionY(),this->_mode_color.getResolutionX(),CV_8UC3,(void*)this->_frame_color->getData());
	////	cv::cvtColor(color_temp,this->_color_mat,CV_RGB2BGR);
	////}

	//if(this->_flag_generators[NIKinect2::NI2_G_IR]){
	//	rc = this->_g_ir->readFrame(this->_frame_ir);

	//	if(rc != openni::STATUS_OK){
	//		NI2_PRINT_ERROR("update_images","Error retrieving IR image.");
	//	}

	//	cv::Mat ir_raw(this->_frame_ir->getHeight(),this->_frame_ir->getWidth(),CV_8UC1,(void*)this->_frame_ir->getData());
	//	ir_raw.copyTo(this->_ir_mat);
	//}
	//printf(" END\n");
	return true;
}

void NIKinect2::analyzeFrame(const openni::VideoFrameRef& frame){
	openni::Status rc = openni::STATUS_OK;

	if(this->_flag_generators[NIKinect2::NI2_G_COLOR]){
		//printf(" read ");
		//rc = frame.readFrame(this->_frame_color);
		//printf(" color ");
		if(rc != openni::STATUS_OK){
			NI2_PRINT_ERROR("update_images","Error retrieving color image.");
			return;
		}

		cv::Mat color_temp(this->_mode_color.getResolutionY(),this->_mode_color.getResolutionX(),CV_8UC3,(void*)frame.getData());
		cv::cvtColor(color_temp,this->_color_mat,CV_RGB2BGR);

		printf("Analyze Frame\n");
	}
}

/**
 * @brief	update_nite.
 * @details	update_nite.
 */
bool NIKinect2::update_nite(){
	nite::Status rc_nite = this->_nite_user_tracker->readFrame(this->_nite_user_tracker_frame);
	
	if (rc_nite != nite::STATUS_OK){
		NI2_PRINT_ERROR("update_nite","User Tracker update failed.");
		return false;
	}

	this->_users->clear();
	this->_users_ids->clear();

	const nite::Array<nite::UserData>& users = this->_nite_user_tracker_frame->getUsers();

	for(int i = 0 ; i < users.getSize() ; ++i){
		//if(users[i].isVisible()){
			if(users[i].isNew()){
				this->_nite_user_tracker->startSkeletonTracking(users[i].getId());
				//printf("start tracking skeleton\n");
			}

			this->_users->push_back(new nite::UserData(users[i]));

			this->_users_ids->push_back(users[i].getId());
		//}
	}

	//printf("NUsers %d\n",this->_nite_user_tracker_frame->getUsers().getSize());
	this->_nite_user_map = (const nite::UserMap&)this->_nite_user_tracker_frame->getUserMap();

	if(this->_flag_generators[NIKinect2::NI2_G_GESTURES]){
		rc_nite = this->_nite_hand_tracker->readFrame(this->_nite_hand_tracker_frame);

		if (rc_nite != nite::STATUS_OK){
			NI2_PRINT_ERROR("update_nite","Hand Tracker update failed.");
			return false;
		}

		//Get gestures from NiTE
		const nite::Array<nite::GestureData>& gestures = this->_nite_hand_tracker_frame->getGestures();

		for(int i = 0; i < gestures.getSize(); ++i){
			if(gestures[i].isComplete()){
				printf("Gesture Complete\n");

				const nite::Point3f& position = gestures[i].getCurrentPosition();
				
				//Start hand tracker
				this->_nite_hand_tracker->startHandTracking(gestures[i].getCurrentPosition(), &this->_nite_hand_id);

				printf("Gesture %d at (%f,%f,%f) ID(%d)\n", gestures[i].getType(), position.x, position.y, position.z,this->_nite_hand_id);
			}
			if(gestures[i].isInProgress()){
				printf("Gesture In Progress\n");
			}
		}

		//Track hand and update position from NiTE
		const nite::Array<nite::HandData>& hands= this->_nite_hand_tracker_frame->getHands();

		printf("N hands (%d)",hands.getSize());
		for(int i = 0; i < hands.getSize(); ++i){
			const nite::HandData& hand_data = hands[i];

			printf(" - %d",hand_data.getId());

			if(hand_data.isTracking()){
				this->convert_3d_to_depth(	hand_data.getPosition().x,
											hand_data.getPosition().y,
											hand_data.getPosition().z,
											&this->xx[i],&this->yy[i],&this->zz[i]);

				//this->_nite_hand_tracker->convertDepthCoordinatesToHand(hand_data.getPosition().x,
				//														hand_data.getPosition().y,
				//														hand_data.getPosition().z,
				//														&this->xx,&this->yy);
				
			}
		}	
		printf("\n");
	}

	return true;
}

bool NIKinect2::calc_floor_plane(){
	if(this->_flag_plane) return true;

	if(this->_flag_generators[NIKinect2::NI2_G_USER]){
		nite::Plane plane = this->_nite_user_tracker_frame->getFloor();

		if(plane.normal.x != 0 || plane.normal.y != 0 || plane.normal.z != 0 ){
			this->_floor_plane->normal = plane.normal;
			this->_floor_plane->point = plane.point;

			this->_floor_plane_a =	this->_floor_plane->normal.x;
			this->_floor_plane_b =	this->_floor_plane->normal.y;
			this->_floor_plane_c =	this->_floor_plane->normal.z;
			this->_floor_plane_d = -(this->_floor_plane->normal.x*this->_floor_plane->point.x + 
									this->_floor_plane->normal.y*this->_floor_plane->point.y + 
									this->_floor_plane->normal.z*this->_floor_plane->point.z);

			this->_flag_plane = true;
		}
		else{
			NI2_PRINT_ERROR("calc_floor_plane","Couldn't get Floor from User Tracker.");
		}
	}
	else{
		NI2_PRINT_ERROR("calc_floor_plane","User Generator not initialized. Please initialize it.");
	}

	return this->_flag_plane;
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
 * @brief	Gets a copy of the IR cv::Mat.
 * @details	Gets a copy of the IR cv::Mat, if the IR Generator 
 *			(_ir_generator) is active.
 *
 * @param[out]	ir
 *				Copy of the IR cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_ir(cv::Mat &ir){
	if(this->_flag_generators[NIKinect2::NI2_G_IR]){
		this->_ir_mat.copyTo(ir);
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
		if(!this->_color_mat.rows)
			return false;

		this->_color_mat.copyTo(color);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	get_users_map.
 * @details	get_users_map.
 *
 * @retval	true if the nite::UserMap was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_users_map(nite::UserMap& map){
	if(this->_flag_generators[NIKinect2::NI2_G_USER]){
		//this->_nite_user_map.opyTo(color);
		map = this->_nite_user_map;
		return true;
	}
	else
		return false;
}

/**
 * @brief	get_users_map.
 * @details	get_users_map.
 *
 * @retval	true if the cv::Mat was successfully created.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect2::get_users_map(cv::Mat &map){
	if(this->_flag_generators[NIKinect2::NI2_G_USER]){
		cv::Mat depthMat16UC1_nite(this->_mode_depth.getResolutionY(),this->_mode_depth.getResolutionX(),CV_16UC1,(void*)this->_nite_user_map.getPixels());
		depthMat16UC1_nite.convertTo(map, CV_8UC1);

		return true;
	}
	else
		return false;
}

/**
 * @brief	get_users_data.
 * @details	get_users_data.
 */
std::vector<nite::UserData*>* NIKinect2::get_users_data(){
	return this->_users;
}

/**
 * @brief	get_users_data.
 * @details	get_users_data.
 */
std::vector<int>* NIKinect2::get_users_ids(){
	return this->_users_ids;
}

/**
 * @brief	get_user_data.
 * @details	get_user_data.
 */
bool NIKinect2::get_user_data(int idx, nite::UserData& data){
	if(!this->_flag_generators[NIKinect2::NI2_G_USER])
		return false;
	
	//const nite::UserData* data_ptr = this->_nite_user_tracker_frame->getUserById(idx);
	if(idx >= this->_users->size())
		return false;

	data = *(new nite::UserData(*this->_users->at(idx)));

	return true;
}

/**
 * @brief	get_user_mask.
 * @details	get_user_mask.
 */
bool NIKinect2::get_user_mask(int idx, cv::Mat &mask){
	if(this->_flag_generators[NIKinect2::NI2_G_USER]){
		cv::Mat depthMat16UC1_nite(480,640,CV_16UC1,(void*)this->_nite_user_map.getPixels());
		cv::inRange(depthMat16UC1_nite,idx,idx,mask);

		return true;
	}
	return false;
}

/**
 * @brief	is_color_hd.
 * @details	is_color_hd.
 */
bool NIKinect2::is_color_hd(){
	if(this->_mode_color.getResolutionX() == 1280)
		return true;
	return false;
}

/**
 * @brief	get_frame_rate.
 * @details	get_frame_rate.
 */
double NIKinect2::get_frame_rate(){
	return this->_frame_rate;
}

//-----------------------------------------------------------------------------
// OPENNI ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	get_device.
 * @details	get_device.
 */
openni::Device*	NIKinect2::get_device(){
	return this->_device;
}

/**
 * @brief	get_depth_stream.
 * @details	get_depth_stream.
 */
openni::VideoStream* NIKinect2::get_depth_stream(){
	return this->_g_depth;
}

/**
 * @brief	get_color_stream.
 * @details	get_color_stream.
 */
openni::VideoStream* NIKinect2::get_color_stream(){
	return this->_g_color;
}

/**
 * @brief	get_ir_stream.
 * @details	get_ir_stream.
 */
openni::VideoStream* NIKinect2::get_ir_stream(){
	return this->_g_ir;
}

/**
 * @brief	get_depth_frame_ref.
 * @details	get_depth_frame_ref.
 */
openni::VideoFrameRef* NIKinect2::get_depth_frame_ref(){
	return this->_frame_depth;
}

/**
 * @brief	get_color_frame_ref.
 * @details	get_color_frame_ref.
 */
openni::VideoFrameRef* NIKinect2::get_color_frame_ref(){
	return this->_frame_color;
}

/**
 * @brief	get_ir_frame_ref.
 * @details	get_ir_frame_ref.
 */
openni::VideoFrameRef* NIKinect2::get_ir_frame_ref(){
	return this->_frame_ir;
}

/**
 * @brief	get_ir_frame_ref.
 * @details	get_ir_frame_ref.
 */
nite::UserTracker* NIKinect2::get_user_tracker(){
	return this->_nite_user_tracker;
}

/**
 * @brief	get_ir_frame_ref.
 * @details	get_ir_frame_ref.
 */
nite::UserTrackerFrameRef* NIKinect2::get_user_tracker_frame(){
	return this->_nite_user_tracker_frame;
}

/**
 * @brief	get_floor_plane.
 * @details	get_floor_plane.
 */
nite::Plane* NIKinect2::get_floor_plane(){
	return (this->_flag_plane) ? this->_floor_plane : NULL;
}

/**
 * @brief	get_floor_plane(double &aa, double &bb, double &cc, double &dd).
 * @details	get_floor_plane(double &aa, double &bb, double &cc, double &dd).
 */
bool NIKinect2::get_floor_plane(double *aa, double *bb, double *cc, double *dd){
	if(!_flag_plane) return false;

	*aa = this->_floor_plane_a;
	*bb = this->_floor_plane_b;
	*cc = this->_floor_plane_c;
	*dd = this->_floor_plane_d;

	return true;
}

/**
 * @brief	get_floor_mask(cv::Mat &out_mask, double thresh).
 * @details	get_floor_mask(cv::Mat &out_mask, double thresh).
 */
bool NIKinect2::get_floor_mask(cv::Mat &out_mask, double thresh){
	if(!this->_flag_plane) return false;

	out_mask = cv::Mat::zeros(480,640,CV_8UC1);

	uint16_t* ptr = (uint16_t*)this->_frame_depth->getData();
	uint8_t* ptr_out = (uint8_t*)out_mask.data;

	float d_x = 0;	float d_y = 0;	float d_z = 0;
	float w_x = 0;	float w_y = 0;	float w_z = 0;	

	for(int y=0; y<480; y++) { 
		for(int x=0; x<640; x++) {
			d_x = x;	d_y = y;	d_z =  ptr[y * 640 + x];
			w_x = 0;	w_y = 0;	w_z = 0;

			openni::CoordinateConverter::convertDepthToWorld(*this->_g_depth,d_x,d_y,d_z,&w_x,&w_y,&w_z);

			float dist = NIKinect2::distance_to_plane(w_x,w_y,w_z,	this->_floor_plane_a,
																	this->_floor_plane_b,
																	this->_floor_plane_c,
																	this->_floor_plane_d);
			if(dist > thresh){
				ptr_out[y * 640 + x] = 255;
			}
		}
	}
}


bool NIKinect2::convert_depth_to_3d(float in_xx, float in_yy, float in_zz, float *out_xx, float *out_yy, float *out_zz){
	return openni::STATUS_OK == openni::CoordinateConverter::convertDepthToWorld(*this->_g_depth,in_xx,in_yy,in_zz,out_xx,out_yy,out_zz);
}

bool NIKinect2::convert_3d_to_depth(float in_xx, float in_yy, float in_zz, float *out_xx, float *out_yy, float *out_zz){
	return openni::STATUS_OK == openni::CoordinateConverter::convertWorldToDepth(*this->_g_depth,in_xx,in_yy,in_zz,out_xx,out_yy,out_zz);
}