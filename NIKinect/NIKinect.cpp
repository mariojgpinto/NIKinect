/** 
 * @file NIKinect.cpp 
 * @author Mario Pinto (mario.pinto@ccg.pt) 
 * @date May, 2013 
 * @brief Implementation of the NIKinect Class.
 */
#include "NIKinect.h"

const char* NIKinect::_sample_xml_path="C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml";

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------
/**
 * @brief	NIKinect Constructor.
 * @details	This constructor 
 *
 * @TODO	Init from ONI File
 */
NIKinect::NIKinect():
	_min_depth(400),
	_max_depth(5000),
	_last_tick(0),
	_frame_counter(0),
	_frame_rate(0) {
	
	for(int i = 0 ; i < (this->_n_flags * this->_n_flags + this->_n_flags) ; ++i)
		this->_flags[i] = false;
}

/** 
 * @brief	NIKinect destructor.
 */
NIKinect::~NIKinect(){

}


//-----------------------------------------------------------------------------
// SETUP
//-----------------------------------------------------------------------------
/** 
 * @brief	NIKinect destructor.
 *
 * @TODO	Init from file_xml
 */
bool NIKinect::init(const char* file, int generators){
	XnStatus rc;

	rc = this->_context.Init();

	if(file){
		rc = this->_context.OpenFileRecording(file);
	}
	else{
		
	}

	int i;
	char flags[this->_n_flags];
	flags[this->_n_flags-1] = '\0';

	for (i = 0; i < this->_n_flags; ++i, generators >>= 1)
	{
		flags[i] = (generators & 1) + '0';
	}

	int ac = 0 ;

	for (i = 0; i < this->_n_flags; ++i){
		if(flags[i]){
			int power = pow(2.0,i);
			switch(power){
				case NIKinect::DEPTH_G:
					ac += (this->init_depth_generator()) ? 0 : NIKinect::DEPTH_G;
					break;
				case NIKinect::IMAGE_G:
					ac += (this->init_image_generator()) ? 0 : NIKinect::IMAGE_G;
					break;
				//case NIKinect::IR_G:
				//	ac += (this->init_ir_generator()) ? 0 : NIKinect::IR_G;
				//	break;
				//case NIKinect::AUDIO_G:
				//	ac += (this->init_audio_generator()) ? 0 : NIKinect::AUDIO_G;
				//	break;
				//case NIKinect::USER_G:
				//	ac += (this->init_user_generator()) ? 0 : NIKinect::USER_G;
				//	break;
				//case NIKinect::GESTURE_G:
				//	ac += (this->init_gesture_generator()) ? 0 : NIKinect::GESTURE_G;
				//	break;
				//case NIKinect::HAND_G:
				//	ac += (this->init_hand_generator()) ? 0 : NIKinect::HAND_G;
				//	break;
				case NIKinect::SCENE_A:
					ac += (this->init_scene_analyzer()) ? 0 : NIKinect::SCENE_A;
					break;
				default: break;
			}
		}
	}

	if(ac){
		return false;
	}
	else{
		
		rc = _depth_generator.GetAlternativeViewPointCap().SetViewPoint(_image_generator);
		rc = this->_context.StartGeneratingAll();
				
		return (rc == XN_STATUS_OK);
	}

	return (bool)ac;
}

/** 
 * @brief	Sets the minimum value for the Depth Range.
 *
 * @param[in]	milimeters
 *				Minimum value for the Depth Range in milimeters.
 */
void NIKinect::set_min_depth(int milimeters){
	this->_min_depth = milimeters;
}

/** 
 * @brief	Sets the maximum value for the Depth Range.
 *
 * @param[in]	milimeters
 *				Maximum value for the Depth Range in milimeters.
 */
void NIKinect::set_max_depth(int milimeters){
	this->_max_depth = milimeters;
}


//-----------------------------------------------------------------------------
// SETUP - INITIALIZE GENERATORS
//-----------------------------------------------------------------------------
/**
 * @brief	Initializes the Depth Generator (_depth_generator).
 *
 * @retval	@c true if the generator was successfully created.
 * @retval	@c false if some error occurred.
 */
bool NIKinect::init_depth_generator(){
	XnStatus rc;
	rc = _context.FindExistingNode(XN_NODE_TYPE_DEPTH,this->_depth_generator);
	//If the generator was already created, don't create it again.
	if (rc != XN_STATUS_OK)
	{
		rc = _depth_generator.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
			this->_flags[NIKinect::DEPTH_G] = false;
		}
		else{
			this->_flags[NIKinect::DEPTH_G] = true;
		}
	}
	else{
		this->_flags[NIKinect::DEPTH_G] = true;
	}

	return this->_flags[NIKinect::DEPTH_G];
}

/**
 * @brief	Initializes the Image Generator (_image_generator).
 *
 * @retval	@c true if the generator was successfully created.
 * @retval	@c false if some error occurred.
 */
bool NIKinect::init_image_generator(){
	XnStatus rc;
	rc = _context.FindExistingNode(XN_NODE_TYPE_IMAGE,this->_image_generator);
	//If the generator was already created, don't create it again.
	if (rc != XN_STATUS_OK)
	{
		rc = _image_generator.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("The Image Node could not be created.%s\n",xnGetStatusString(rc));
			this->_flags[NIKinect::IMAGE_G] = false;
		}
		else{
			this->_flags[NIKinect::IMAGE_G] = true;
		}
	}
	else{
		this->_flags[NIKinect::IMAGE_G] = true;
	}

	return this->_flags[NIKinect::IMAGE_G];
}

/**
 * @brief	Initializes the Scene Analyzer (_scene_analyzer).
 *
 * @retval	@c true if the Scene Analyzer was successfully created.
 * @retval	@c false if some error occurred.
 */
bool NIKinect::init_scene_analyzer(){
	XnStatus rc;
	rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE,this->_scene_analyzer);
	//If the generator was already created, don't create it again.
	if (rc != XN_STATUS_OK)
	{
		rc = _scene_analyzer.Create(_context);

		if (rc != XN_STATUS_OK)
		{
			printf("The Scene Analyzer Node could not be created.%s\n",xnGetStatusString(rc));
			this->_flags[NIKinect::SCENE_A] = false;
		}
		else{
			this->_flags[NIKinect::SCENE_A] = true;
		}
	}
	else{
		this->_flags[NIKinect::SCENE_A] = true;
	}

	return this->_flags[NIKinect::SCENE_A];
}

//-----------------------------------------------------------------------------
// RUNNING
//-----------------------------------------------------------------------------
/**
 * @brief	Updates the generator's information.
 * @details	.
 *
 * @TODO	Implement Other Generators and SceneAnalyzer update.
 */
bool NIKinect::update(){
	XnStatus rc;
	rc = _context.WaitAnyUpdateAll();

	this->update_frame_rate();

	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		//break;
		return false;
	}

	//Updates Depth Variables
	if(this->_flags[NIKinect::DEPTH_G]){
		this->_depth_generator.GetMetaData(_depth_md);
		cv::Mat depthMat16UC1(480, 640,CV_16UC1, (void*) _depth_md.Data());
		depthMat16UC1.copyTo(_depth_mat);

		cv::inRange(depthMat16UC1,1,5000,_mask_mat);

		double min = this->_min_depth;
		double max = this->_max_depth;
		NIKinect::compute_color_encoded_depth(this->_depth_mat,this->_depth_as_color_mat,&min,&max);
	}

	//Updates Image Variables
	if(this->_flags[NIKinect::IMAGE_G]){
		this->_image_generator.GetMetaData(_image_md);
		cv::Mat color_temp(480,640,CV_8UC3,(void*) _image_md.Data());
		cv::cvtColor(color_temp,_color_mat,CV_RGB2BGR);
	}

	if(this->_flags[NIKinect::SCENE_A]){
		
	}
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------
/**
 * @brief	Updates the frame rate variables.
 */
void NIKinect::update_frame_rate(){
	++this->_frame_counter;
	if (this->_frame_counter == 10)
	{
		double current_tick = cv::getTickCount();
		this->_frame_rate = _frame_counter / ((current_tick - this->_last_tick)/cv::getTickFrequency());
		this->_last_tick = current_tick;
		this->_frame_counter = 0;
	}
}

/**
 * @brief	Computes a Colorized Image from a Depth Image within a given range.
 */
void NIKinect::compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat& color_depth_im,
                                     double* i_min_val, double* i_max_val){
	double min_val, max_val;
	if (i_min_val && i_max_val)
	{
		min_val = *i_min_val;
		max_val = *i_max_val;
	}
	else
	{
		minMaxLoc(depth_im, &min_val, &max_val);
	}

	color_depth_im.create(depth_im.size(),CV_8UC3);
	for (int r = 0; r < depth_im.rows; ++r)
	{
		const float* depth_data = depth_im.ptr<float>(r);
		cv::Vec3b* depth_color_data = color_depth_im.ptr<cv::Vec3b>(r);
		for (int c = 0; c < depth_im.cols; ++c)
		{
			int v = 255*6*(depth_data[c]-min_val)/(max_val-min_val);
			if (v < 0) v = 0;
			char r,g,b;
			int lb = v & 0xff;
			switch (v / 256) {
			case 0:
				r = 255;	g = 255-lb;	b = 255-lb;
				break;
			case 1:
				r = 255;	g = lb;		b = 0;
				break;
			case 2:
				r = 255-lb;	g = 255;	b = 0;
				break;
			case 3:
				r = 0;		g = 255;	b = lb;
				break;
			case 4:
				r = 0;		g = 255-lb;	b = 255;
				break;
			case 5:
				r = 0;		g = 0;		b = 255-lb;
				break;
			default:
				r = 0;		g = 0;		b = 0;
				break;
			}
			if (v == 0){
				r = g = b = 0;
			}
			depth_color_data[c] = cv::Vec3b(b,g,r);
		}
	}
}

/**
 * @brief	Calculates the Floor Plane, if possible.
 *
 * @param[out]	a
 *				'a' value of the plane equation.
 * @param[out]	b
 *				'b' value of the plane equation.
 * @param[out]	c
 *				'c' value of the plane equation.
 * @param[out]	d
 *				'd' value of the plane equation.
 *
 * @retval	@c true if the cv::Mat was successfully copied.
 * @retval	@c false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_floor_plane(double *a, double *b, double *c, double *d){
	if(this->_flags[NIKinect::SCENE_A]){
		XnStatus rc;
		XnPlane3D floorCoords;

		rc = this->_scene_analyzer.GetFloor(floorCoords);
		
		if(rc == XN_STATUS_OK){
			*a = floorCoords.vNormal.X;
			*b = floorCoords.vNormal.Y;
			*c = floorCoords.vNormal.Z;
			*d = -(	floorCoords.vNormal.X * floorCoords.ptPoint.X + 
					floorCoords.vNormal.Y * floorCoords.ptPoint.Y + 
					floorCoords.vNormal.Z * floorCoords.ptPoint.Z);

			return true;
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
}

/**
 * @brief	Calculates the Floor Plane, if possible.
 *
 * @param[out]	mask
 *				Mask with the points of the floor.
 *
 * @param[out]	a
 *				'a' value of the plane equation.
 * @param[out]	b
 *				'b' value of the plane equation.
 * @param[out]	c
 *				'c' value of the plane equation.
 * @param[out]	d
 *				'd' value of the plane equation.
 */
//void NIKinect::remove_plane(cv::Mat& mask, double a, double b, double c, double d){
//	if(this->_flags[NIKinect::SCENE_A]){
//		XnStatus rc;
//		XnPlane3D floorCoords;
//		XnPoint3D floorPoint;
//		XnPoint3D * pointList = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
//		XnPoint3D * realWorld = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
//
//		return true;
//	}
//	else{
//		return false;
//	}
//}

//-----------------------------------------------------------------------------
// ACCESS
//-----------------------------------------------------------------------------
/**
 * @brief	Gives the Frame Rate of the running NIKinect.
 *
 * @return	The current frame rate.
 */
double NIKinect::get_frame_rate(){
	return this->_frame_rate;
}

//-----------------------------------------------------------------------------
// ACCESS - GENERATORS 
//-----------------------------------------------------------------------------
/**
 * @brief	Access to the current NI Context.
 * @return	The NI Context.
 */
xn::Context& NIKinect::get_context(){
	return this->_context;
}

/**
 * @brief	Access to the Depth Generator.
 * @return	The Depth Generator.
 */
xn::DepthGenerator& NIKinect::get_depth_generator(){
	return this->_depth_generator;
}

/**
 * @brief	Access to the Image Generator.
 * @return	The Image Generator.
 */
xn::ImageGenerator& NIKinect::get_image_generator(){
	return this->_image_generator;
}

/**
 * @brief	Access to the IR Generator.
 * @return	The IR Generator.
 */
xn::IRGenerator& NIKinect::get_ir_generator(){
	return this->_ir_generator;
}

/**
 * @brief	Access to the Audio Generator.
 * @return	The Audio Generator.
 */
xn::AudioGenerator& NIKinect::get_audio_generator(){
	return this->_audio_generator;
}

/**
 * @brief	Access to the User Generator.
 * @return	The User Generator.
 */
xn::UserGenerator& NIKinect::get_user_generator(){
	return this->_user_generator;
}

/**
 * @brief	Access to the Gesture Generator.
 * @return	The Gesture Generator.
 */
xn::GestureGenerator& NIKinect::get_gesture_generator(){
	return this->_gesture_generator;
}

/**
 * @brief	Access to the Hands Generator.
 * @return	The Hands Generator.
 */
xn::HandsGenerator& NIKinect::get_hands_generator(){
	return this->_hands_generator;
}

/**
 * @brief	Access to the Scene Analyzer.
 * @return	The Scene Analyzer.
 */
xn::SceneAnalyzer& NIKinect::get_scene_analyzer(){
	return this->_scene_analyzer;
}

//-----------------------------------------------------------------------------
// ACCESS - OPENCV MAT
//-----------------------------------------------------------------------------
/**
 * @brief	Gets a copy of the Depth cv::Mat.
 * @details	Gets a copy of the Depth cv::Mat, if the Depth Generator 
 *			(_depth_generator) is active.
 *
 * @param[out]	depth
 *				Copy of the Depth cv::Mat.
 *
 * @retval	@c true if the cv::Mat was successfully copied.
 * @retval	@c false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_depth(cv::Mat &depth){
	if(this->_flags[NIKinect::DEPTH_G]){
		this->_depth_mat.copyTo(depth);
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
 * @retval	@c true if the cv::Mat was successfully copied.
 * @retval	@c false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_mask(cv::Mat &mask){
	if(this->_flags[NIKinect::DEPTH_G]){
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
 * @retval	@c true if the cv::Mat was successfully copied.
 * @retval	@c false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_color(cv::Mat &color){
	if(this->_flags[NIKinect::IMAGE_G]){
		this->_color_mat.copyTo(color);
		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	Gets a colorized image of the Depth cv::Mat, according to the given range. 
 * @details	Gets a colorized image of the Depth cv::Mat, according to the given range,
 *			if the Depth Generator (_depth_generator) is active.
 *			If the range is not given by the user, it will use the NIKinect default
 *			values (_min_depth and _max_depth).
 *			@see compute_color_encoded_depth.
 *
 * @param[out]	depth_as_color
 *				Colorized image of the Depth cv::Mat, according to the given range.
 *
 * @param[in]	min
 *				Minimum depth (in milimeters).
 *
 * @param[in]	max
 *				Maximum depth (in milimeters).
 *
 * @retval	@c true if the cv::Mat was successfully created.
 * @retval	@c false if the generator is not active or some other error occurred.
 */
//bool NIKinect::get_depth_as_color(cv::Mat &depth_as_color, int min, int max){
//	if(this->_flags[NIKinect::DEPTH_G]){
//		double min_t = (min == -1) ? this->_min_depth : min;
//		double max_t = (max == -1) ? this->_max_depth : max;
//
//		NIKinect::compute_color_encoded_depth(this->_depth_mat,depth_as_color,&min_t,&max_t);
//
//		return true;
//	}
//	else{
//		return false;
//	}
//}
bool NIKinect::get_depth_as_color(cv::Mat3b &depth_as_color){
	if(this->_flags[NIKinect::DEPTH_G]){
		this->_depth_as_color_mat.copyTo(depth_as_color);
		return true;
	}
	else{
		return false;
	}
}

bool NIKinect::get_depth_meta_data(xn::DepthMetaData *depth){
	if(this->_flags[NIKinect::DEPTH_G]){
		depth = &this->_depth_md;
		return true;
	}
	else{
		return false;
	}
}