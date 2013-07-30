/** 
 * @file	NIKinect.cpp 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	May, 2013 
 * @brief	Implementation of the NIKinect Class.
 *
 * @details	Deailed Information.
 */
#include "NIKinect.h"

//const char* NIKinect::_sample_xml_path="C:\\Dev\\External\\OpenNI\\Data\\SamplesConfig.xml";

//-----------------------------------------------------------------------------
// CONSTRUCTORS
//-----------------------------------------------------------------------------

/**
 * @brief	NIKinect Constructor.
 * @details	This constructor 
 *
 * @todo	Init from ONI File
 */
NIKinect::NIKinect():
	_min_depth(400),
	_max_depth(5000),
	_last_tick(0),
	_frame_counter(0),
	_frame_rate(0),
	_point_step(2) {
	
	for(int i = 0 ; i < (this->_n_flags * this->_n_flags + this->_n_flags) ; ++i)
		this->_flags[i] = false;

	for(int i = 0 ; i < (this->_n_processing * this->_n_processing + this->_n_processing) ; ++i)
		this->_flags_processing[i] = false;

	this->_point_2d = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	this->_point_3d = (XnPoint3D *)malloc(sizeof(XnPoint3D) * XN_VGA_Y_RES * XN_VGA_X_RES); 
	
	//this->_cloud_pcl.width = XN_VGA_Y_RES * XN_VGA_X_RES;
	//this->_cloud_pcl.height = 1;
	//this->_cloud_pcl.points.resize (this->_cloud_pcl.width * this->_cloud_pcl.height);
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
 * @brief	NIKinect Initializer.
 *
 * @todo	Init from file_xml.
 * @todo	Validation of number of nodes.
 */
bool NIKinect::init(const char* file, int n_kinect, int generators){
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
		if(flags[i] == '1'){
			int power = pow(2.0,i);
			switch(power){
				case NIKinect::DEPTH_G:
					ac += (this->init_depth_generator(n_kinect)) ? 0 : NIKinect::DEPTH_G;
					break;
				case NIKinect::IMAGE_G:
 					ac += (this->init_image_generator(n_kinect)) ? 0 : NIKinect::IMAGE_G;
					break;
				//case NIKinect::IR_G:
				//	ac += (this->init_ir_generator()) ? 0 : NIKinect::IR_G;
				//	break;
				//case NIKinect::AUDIO_G:
				//	ac += (this->init_audio_generator()) ? 0 : NIKinect::AUDIO_G;
				//	break;
				case NIKinect::USER_G:
					ac += (this->init_user_generator(n_kinect)) ? 0 : NIKinect::USER_G;
					break;
				//case NIKinect::GESTURE_G:
				//	ac += (this->init_gesture_generator()) ? 0 : NIKinect::GESTURE_G;
				//	break;
				//case NIKinect::HAND_G:
				//	ac += (this->init_hand_generator()) ? 0 : NIKinect::HAND_G;
				//	break;
				case NIKinect::SCENE_A:
					ac += (this->init_scene_analyzer(n_kinect)) ? 0 : NIKinect::SCENE_A;
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
 * @brief		Sets the minimum value for the Depth Range.
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

/** 
 * @brief	Sets the Processing Flag's value to the given value (true or false).
 *
 * @param[in]	flag
 *				Flag to be set the value
 * @param[in]	value
 *				Value to be set to the flag.
 */
void NIKinect::set_processing_flag(NIKinect::PROCESSING flag, bool value){
	this->_flags_processing[flag] = value;
}

/** 
 * @brief		Sets the step value for the analysis of the Depth image to 3D point generation.
 *
 * @param[in]	step
 *				Value of the analysis step.
 */
void NIKinect::set_3d_analysis_step(int step){
	this->_point_step = step;
}

//-----------------------------------------------------------------------------
// SETUP - INITIALIZE GENERATORS
//-----------------------------------------------------------------------------
/**
 * @brief	Initializes the Depth Generator (_depth_generator).
 *
 * @retval	true if the generator was successfully created.
 * @retval	false if some error occurred.
 */
bool NIKinect::init_depth_generator(int index){
	static xn::NodeInfoList depth_nodes;
	XnStatus rc;
	rc = this->_context.EnumerateProductionTrees (XN_NODE_TYPE_DEPTH, NULL, depth_nodes, NULL);

	int counter = 0;
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, counter++) {
		if(counter == index){
			xn::NodeInfo info = *nodeIt;
			const XnProductionNodeDescription& description = info.GetDescription();
	
			XnMapOutputMode mode;
			mode.nXRes	= 640;
			mode.nYRes	= 480;
			mode.nFPS	= 30;

			rc = this->_context.CreateProductionTree (info);
		
			this->_depth_generator;// = new xn::DepthGenerator();
			//DepthMetaData* g_depthMD = new DepthMetaData();

			rc = info.GetInstance (this->_depth_generator);
		
			if (rc != XN_STATUS_OK)
			{
				printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
				this->_flags[NIKinect::DEPTH_G] = false;
				this->_flags_processing[NIKinect::DEPTH_P] = false;
				this->_flags_processing[NIKinect::MASK_P] = false;

			}
			else{
				this->_flags[NIKinect::DEPTH_G] = true;
				this->_flags_processing[NIKinect::DEPTH_P] = true;
				this->_flags_processing[NIKinect::MASK_P] = true;
			}

			this->_depth_generator.SetMapOutputMode(mode);
			//this->_depth_generator.GetMetaData(this->_depth_md);
			this->_depth_generator.StartGenerating();

			break;
		}
	}

	if(this->_flags[NIKinect::DEPTH_G]){
		this->_depth_generator.GetMetaData(this->_depth_md);
	}

	//rc = _context.FindExistingNode(XN_NODE_TYPE_DEPTH,this->_depth_generator);
	////If the generator was already created, don't create it again.
	//if (rc != XN_STATUS_OK)
	//{
	//	rc = _depth_generator.Create(_context);

	//	if (rc != XN_STATUS_OK)
	//	{
	//		printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
	//		this->_flags[NIKinect::DEPTH_G] = false;
	//		this->_flags_processing[NIKinect::DEPTH_P] = false;
	//		this->_flags_processing[NIKinect::MASK_P] = false;

	//	}
	//	else{
	//		this->_flags[NIKinect::DEPTH_G] = true;
	//		this->_flags_processing[NIKinect::DEPTH_P] = true;
	//		this->_flags_processing[NIKinect::MASK_P] = true;
	//	}
	//}
	//else{
	//	this->_flags[NIKinect::DEPTH_G] = true;
	//	this->_flags_processing[NIKinect::DEPTH_P] = true;
	//	this->_flags_processing[NIKinect::MASK_P] = true;
	//}

	return this->_flags[NIKinect::DEPTH_G];
}

/**
 * @brief	Initializes the Image Generator (_image_generator).
 *
 * @retval	true if the generator was successfully created.
 * @retval	false if some error occurred.
 */
bool NIKinect::init_image_generator(int index){
	static xn::NodeInfoList depth_nodes;
	XnStatus rc;
	rc = this->_context.EnumerateProductionTrees (XN_NODE_TYPE_IMAGE, NULL, depth_nodes, NULL);

	int counter = 0;
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, counter++) {
		if(counter == index){
			xn::NodeInfo info = *nodeIt;
			const XnProductionNodeDescription& description = info.GetDescription();
	
			XnMapOutputMode mode;
			mode.nXRes	= 640;
			mode.nYRes	= 480;
			mode.nFPS	= 30;

			rc = this->_context.CreateProductionTree (info);

			rc = info.GetInstance (this->_image_generator);
		
			if (rc != XN_STATUS_OK)
			{
				printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
				this->_flags[NIKinect::IMAGE_G] = false;
				this->_flags_processing[NIKinect::IMAGE_P] = false;

			}
			else{
				this->_flags[NIKinect::IMAGE_G] = true;
				this->_flags_processing[NIKinect::IMAGE_P] = true;
			}

			this->_image_generator.SetMapOutputMode(mode);
			//this->_image_generator.GetMetaData(this->_image_md);
			this->_image_generator.StartGenerating();

			break;
		}
	}

	if(this->_flags[NIKinect::IMAGE_G]){
		this->_image_generator.GetMetaData(this->_image_md);	
		if(this->_flags[NIKinect::DEPTH_G]){
			this->_depth_generator.GetAlternativeViewPointCap().SetViewPoint(this->_image_generator);
		}
	}

	//XnStatus rc;
	//rc = _context.FindExistingNode(XN_NODE_TYPE_IMAGE,this->_image_generator);
	////If the generator was already created, don't create it again.
	//if (rc != XN_STATUS_OK)
	//{
	//	rc = _image_generator.Create(_context);

	//	if (rc != XN_STATUS_OK)
	//	{
	//		printf("The Image Node could not be created.%s\n",xnGetStatusString(rc));
	//		this->_flags[NIKinect::IMAGE_G] = false;
	//		this->_flags_processing[NIKinect::IMAGE_P] = false;
	//	}
	//	else{
	//		this->_flags[NIKinect::IMAGE_G] = true;
	//		this->_flags_processing[NIKinect::IMAGE_P] = true;
	//	}
	//}
	//else{
	//	this->_flags[NIKinect::IMAGE_G] = true;
	//	this->_flags_processing[NIKinect::IMAGE_P] = true;
	//}

	//if(this->_flags[NIKinect::IMAGE_G]){
	//	this->_image_generator.GetMetaData(this->_image_md);		
	//	this->_depth_generator.GetAlternativeViewPointCap().SetViewPoint(this->_image_generator);
	//}

	return this->_flags[NIKinect::IMAGE_G];
}

/**
 * @brief	Initializes the Scene Analyzer (_scene_analyzer).
 *
 * @retval	true if the Scene Analyzer was successfully created.
 * @retval	false if some error occurred.
 */
bool NIKinect::init_scene_analyzer(int index){
	static xn::NodeInfoList depth_nodes;
	XnStatus rc;
	rc = this->_context.EnumerateProductionTrees (XN_NODE_TYPE_SCENE, NULL, depth_nodes, NULL);

	int counter = 0;
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, counter++) {
		if(counter == index){
			xn::NodeInfo info = *nodeIt;
			const XnProductionNodeDescription& description = info.GetDescription();

			rc = this->_context.CreateProductionTree (info);

			rc = info.GetInstance (this->_scene_analyzer);
		
			if (rc != XN_STATUS_OK)
			{
				printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
				this->_flags[NIKinect::SCENE_A] = false;

			}
			else{
				this->_flags[NIKinect::SCENE_A] = true;
			}

			break;
		}
	}
	
	if(this->_flags[NIKinect::SCENE_A]){
		this->_scene_analyzer.GetMetaData(this->_scene_md);
	}

	//XnStatus rc = 1;
	////rc = _context.FindExistingNode(XN_NODE_TYPE_SCENE,this->_scene_analyzer);
	////If the generator was already created, don't create it again.
	//if (rc != XN_STATUS_OK)
	//{
	//	rc = _scene_analyzer.Create(_context);

	//	if (rc != XN_STATUS_OK)
	//	{
	//		printf("The Scene Analyzer Node could not be created.%s\n",xnGetStatusString(rc));
	//		this->_flags[NIKinect::SCENE_A] = false;
	//	}
	//	else{
	//		this->_flags[NIKinect::SCENE_A] = true;
	//	}
	//}
	//else{
	//	this->_flags[NIKinect::SCENE_A] = true;
	//}
	
	return this->_flags[NIKinect::SCENE_A];
}

/**
 * @brief	Initializes the User Generator(_user_generator).
 *
 * @retval	true if the User Generator was successfully created.
 * @retval	false if some error occurred.
 */
bool NIKinect::init_user_generator(int index){
	static xn::NodeInfoList depth_nodes;
	XnStatus rc;
	rc = this->_context.EnumerateProductionTrees (XN_NODE_TYPE_USER, NULL, depth_nodes, NULL);

	int counter = 0;
	for (xn::NodeInfoList::Iterator nodeIt =depth_nodes.Begin(); nodeIt != depth_nodes.End(); ++nodeIt, counter++) {
		if(counter == index){
			xn::NodeInfo info = *nodeIt;
			const XnProductionNodeDescription& description = info.GetDescription();

			rc = this->_context.CreateProductionTree (info);

			rc = info.GetInstance (this->_scene_analyzer);
		
			if (rc != XN_STATUS_OK)
			{
				printf("The Depth Node could not be created.%s\n",xnGetStatusString(rc));
				this->_flags[NIKinect::USER_G] = false;

			}
			else{
				this->_flags[NIKinect::USER_G] = true;
			}

			break;
		}
	}
	
	if(this->_flags[NIKinect::USER_G]){
		this->_scene_analyzer.GetMetaData(this->_scene_md);
	}

	//XnStatus rc = 1;
	//rc = _context.FindExistingNode(XN_NODE_TYPE_USER,this->_user_generator);
	////If the generator was already created, don't create it again.
	//if (rc != XN_STATUS_OK)
	//{
	//	rc = _user_generator.Create(_context);

	//	if (rc != XN_STATUS_OK)
	//	{
	//		printf("The Scene Analyzer Node could not be created.%s\n",xnGetStatusString(rc));
	//		this->_flags[NIKinect::USER_G] = false;
	//	}
	//	else{
	//		this->_flags[NIKinect::USER_G] = true;
	//	}
	//}
	//else{
	//	this->_flags[NIKinect::USER_G] = true;
	//}

	return this->_flags[NIKinect::USER_G];
}

//-----------------------------------------------------------------------------
// RUNNING
//-----------------------------------------------------------------------------
/**
 * @brief	Updates the generator's information.
 * @details	Use it in single-threaded applications.
 *
 * @todo	Implement Other Generators and SceneAnalyzer update.
 */
bool NIKinect::update(){
	if(this->update_openni()){
		return this->update_images();
	}
	return false;
}

/**
 * @brief	Updates the NIKinect's Metda-Data and Image information.
 * @details	When called on multi-threaded environment, use it inside locks.
 *
 * @retval	true if the information was successfully updated.
 * @retval	false if an error occurred.
 */
bool NIKinect::update_images(){
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

	if(this->_flags[NIKinect::SCENE_A]){
		
	}

	if(this->_flags_processing[NIKinect::POINT_CLOUD]){
		this->generate_point_cloud();
	}

	this->update_frame_rate();

	return true;
}

/**
 * @brief	Updates the generator's information
 * @details	When called on multi-threaded environment, use it outside locks.
 *
 * @retval	true if generators where successfully updated.
 * @retval	false if an error occurred.
 */
bool NIKinect::update_openni(){
	XnStatus rc;
	rc = _context.WaitAnyUpdateAll();

	if (rc != XN_STATUS_OK)
	{
		printf("Read failed: %s\n", xnGetStatusString(rc));
		//break;
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
// PROCESSING
//-----------------------------------------------------------------------------

/**
 * @brief	Generates the 3D point cloud of the scene.
 * @details	
 */
void NIKinect::generate_point_cloud(){
	int ac = 0;
	for(int y=0; y<XN_VGA_Y_RES; y+=_point_step) { 
		for(int x=0; x<XN_VGA_X_RES; x+=_point_step) { 
			XnPoint3D point1;
			point1.X = x; 
			point1.Y = y; 
			point1.Z = _depth_md[y * XN_VGA_X_RES + x]; 

			//this->_point_2d[y * XN_VGA_X_RES + x] = point1;
			this->_point_2d[ac++] = point1;
		}
	} 
	//int n = ;
	_depth_generator.ConvertProjectiveToRealWorld((XN_VGA_Y_RES*XN_VGA_X_RES) / (float)(_point_step*_point_step), this->_point_2d, this->_point_3d);
}

/**
 * @brief		Converts a list of points from projective coordinates to real world coordinates.
 * @details	
 *
 * @param[in]	count
 *				Number of points to be converted.
 * @param[in]	points_in
 *				List of points to be converted.
 * @param[out]	points_out
 *				List of converted points.
 *
 * @retval		true if the points were successfully converted.
 * @retval		false if the input was invalid or an error occurred.
 */
bool NIKinect::convert_to_realworld(int count, XnPoint3D* points_in, XnPoint3D* points_out){
	if(count == 0) return true;
	if(!points_in) return false;

	if(!points_out) 
		points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * count);

	XnStatus result = this->_depth_generator.ConvertProjectiveToRealWorld(count,points_in,points_out);

	return result == XN_STATUS_OK;
}

/**
 * @brief		Converts a list of points from projective coordinates to real world coordinates.
 * @details	
 *
 * @param[in]	mask
 *				Mask to filter the Depth Map.
 * @param[out]	points_out
 *				List with the converted points in realworld 3D Coordinates.
 * @param[in]	min_x
 *				Minimum X value of the mask.
 * @param[in]	max_x
 *				Maximum X value of the mask.
 * @param[in]	min_y
 *				Minimum Y value of the mask.
 * @param[in]	max_y
 *				Maximum Y value of the mask.
 *
 * @retval		true if the points were successfully converted.
 * @retval		false if the input was invalid or an error occurred.
 */
bool NIKinect::convert_to_realworld(cv::Mat mask, XnPoint3D* points_out, int min_x, int max_x, int min_y, int max_y){
	if(mask.size().width != this->_depth_mat.size().width || mask.size().height != this->_depth_mat.size().height || mask.type() != CV_8UC1) return false;

	int _min_x,_max_x,_min_y,_max_y;

	if(min_x >= -1 && min_x >= -1 && min_x >= -1 && min_x >= -1){
		_min_x = min_x; _max_x = max_x; _min_y = min_y; _max_y = max_y;
	}
	else{
		_min_x = 0; _max_x = mask.cols; _min_y = 0; _max_y = mask.rows;
	}

	int total_points = (_max_x - _min_x) * (_max_y - _min_y);
	XnPoint3D* points_in = (XnPoint3D*)malloc(sizeof(XnPoint3D) * total_points);
	
	if(points_out == NULL) points_out = (XnPoint3D*)malloc(sizeof(XnPoint3D) * total_points);

	uchar* ptr_mask = mask.data;
	int mask_step = mask.cols;
	int n_points = 0;
	for(int x = _min_x; x < _max_x ; x+=_point_step){
		for(int y = _min_y ; y < _max_y ; y+=_point_step){
			if(ptr_mask[y * mask_step + x]){
				XnPoint3D point1;
				point1.X = x; 
				point1.Y = y; 
				point1.Z = this->_depth_md[y * mask_step + x]; 

				points_in[n_points] = point1;
				n_points++;
			}
		}
	}

	XnStatus result = this->_depth_generator.ConvertProjectiveToRealWorld(n_points,points_in,points_out);

	return result == XN_STATUS_OK;
}

/**
 * @brief	Updates the frame rate variables.
 */
void NIKinect::update_frame_rate(){
	++this->_frame_counter;
	if (this->_frame_counter == 5)
	{
		int64 current_tick = cv::getTickCount();
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
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
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

/*
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
 * @brief	Gets a copy of the Color cv::Mat.
 * @details	Gets a copy of the Color cv::Mat, if the Image Generator 
 *			(_image_generator) is active.
 *
 * @param[out]	depth
 *				Copy of the Image cv::Mat.
 *
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_depth_meta_data(xn::DepthMetaData& depth){
	if(this->_flags[NIKinect::DEPTH_G]){
		depth.CopyFrom(this->_depth_md);
		return true;
	}
	else{
		return false;
	}
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
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_depth(cv::Mat &depth){
	if(this->_flags[NIKinect::DEPTH_G] && this->_flags_processing[NIKinect::DEPTH_P]){
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
 * @retval	true if the cv::Mat was successfully copied.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_mask(cv::Mat &mask){
	if(this->_flags[NIKinect::DEPTH_G] && this->_flags_processing[NIKinect::MASK_P]){
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
bool NIKinect::get_color(cv::Mat &color){
	if(this->_flags[NIKinect::IMAGE_G && this->_flags_processing[NIKinect::IMAGE_P]]){
		this->_color_mat.copyTo(color);
		return true;
	}
	else{
		return false;
	}
}

/*
 * @brief	Gets a colorized image of the Depth cv::Mat, according to the given range. 
 * @details	if the Depth Generator (_depth_generator) is active.
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
 * @retval	true if the cv::Mat was successfully created.
 * @retval	false if the generator is not active or some other error occurred.
 */
//bool NIKinect::get_depth_as_color(cv::Mat &depth_as_color, int min, int max){
//	if(this->_flags[NIKinect::DEPTH_G] && this->_flags_processing[NIKinect::DEPTH_COLOR]){
//		double min_t = (min == -1) ? this->_min_depth : min;
//		double max_t = (max == -1) ? this->_max_depth : max;
//
//		NIKinect::compute_color_encoded_depth(this->_depth_mat,depth_as_color,&min_t,&max_t);
//
//		return true;
//	}
//	else{
//		return false;
//	}@link DEPTH_G \endlink
//}
/**
 * @brief	Gets a colorized image of the Depth cv::Mat, according to the NIKinect's range. 
 * @details	The #DEPTH_G Generation Flag and the DEPTH_COLOR Processing flag must be active.
 *			The used range is set by the _min_depth and _max_depth values.
 *			@see compute_color_encoded_depth.
 *
 * @param[out]	depth_as_color
 *				Colorized image of the Depth cv::Mat, according to the given range.
 *
 * @retval	true if the cv::Mat was successfully created.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_depth_as_color(cv::Mat &depth_as_color){
	if(this->_flags[NIKinect::DEPTH_G] && this->_flags_processing[NIKinect::DEPTH_COLOR]){
		this->_depth_as_color_mat.copyTo(depth_as_color);
		return true;
	}
	else{
		return false;
	}
}


/**
 * @brief	Gets the depth mask, according to the given range. 
 * @details	Gets the depth mask, according to the given range. 
 *			If the range is not given by the user, it will use the NIKinect default
 *			values (_min_depth and _max_depth).
 *
 * @param[out]	mask
 *				Depth Mask, according to the given range.
 *
 * @param[in]	min
 *				Minimum depth (in milimeters).
 *
 * @param[in]	max
 *				Maximum depth (in milimeters).
 *
 * @retval	true if the cv::Mat was successfully created.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_range_mask(cv::Mat &mask, int min, int max){
	if(this->_flags[NIKinect::DEPTH_G]){
		int min_t = (min == -1) ? this->_min_depth : min;
		int max_t = (max == -1) ? this->_max_depth : max;

		cv::inRange(this->_depth_mat,min_t,max_t,mask);

		return true;
	}
	else{
		return false;
	}
}

/**
 * @brief	Gets the depth cv::Mat, according to the given range. 
 * @details	If the range is not given by the user, it will use the NIKinect default
 *			values (_min_depth and _max_depth).
 *
 * @param[out]	depth
 *				Depth cv::Mat, according to the given range.
 *
 * @param[in]	min
 *				Minimum depth (in milimeters).
 *
 * @param[in]	max
 *				Maximum depth (in milimeters).
 *
 * @retval	true if the cv::Mat was successfully created.
 * @retval	false if the generator is not active or some other error occurred.
 */
bool NIKinect::get_range_depth(cv::Mat &depth, int min, int max){
	if(this->_flags[NIKinect::DEPTH_G]){
		int min_t = (min == -1) ? this->_min_depth : min;
		int max_t = (max == -1) ? this->_max_depth : max;

		cv::Mat mask;
		cv::inRange(this->_depth_mat,min_t,max_t,mask);

		this->_depth_mat.copyTo(depth,mask);

		return true;
	}
	else{
		return false;
	}
}
//
//bool get_range_color(cv::Mat &color, int min = -1, int max = -1);

/**
 * @brief	Returns the pointer to the list of 3D points.
 * @details	
 *
 * @return	The pointer to the list. @c NULL if the list is not calculated.
 */
XnPoint3D* NIKinect::get_points_3d(){
	return (this->_flags_processing[NIKinect::POINT_CLOUD]) ? this->_point_3d : NULL;
}

/**
 * @brief	Returns the current 3D analysis step.
 * @details	@see _point_step.
 *
 * @return	The current 3D analysis step.
 *
 */
int NIKinect::get_3d_analysis_step(){
	return this->_point_step;
}


