/** 
 * @file	NIKinect2.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	October, 2013 
 * @brief	Declaration of the NIKinect2 Class.
 */

#ifndef _NI_KINECT_2
#define _NI_KINECT_2

#include <OpenNI.h>
#include <NiTE.h>
#include <opencv2\opencv.hpp>

#define NI2_PRINT_ERROR(function, message)									\
	printf("NIKINECT2 ERROR\n");											\
	printf("Function: %s\n",(function) ? function : "UNKOWN");				\
	printf("Internal Message: %s\n", (message) ? message : "NONE");			\
	printf("OpenNI Description: %s\n", openni::OpenNI::getExtendedError());
	

/**
 * @class NIKinect2
 * @brief	Wrapper for OpenNI2 to use Kinect.
 * @details	
 *
 * @TODO	- Add Depth / Color Sync
 *			- Add Floor calc and plane retrieve
 *			- Add Depth Control
 *			- Add Image Size Control (Update Generators)
 *			- Add 3D Convertion
 *			- Add access to Generators
 *			- Add Color Depth
 *			- Add User Update
 *			- Add User Retrieve (mask, bbox, center of mass)
 **/
class __declspec(dllexport)NIKinect2{
	public:
		enum GENERATORS{
			NI2_G_DEPTH = 0,
			NI2_G_COLOR = 1,
			NI2_G_IR = 2
		};
		static const int _n_generators = 3;

	public:
		NIKinect2();
		~NIKinect2();

		static bool ni_initialize();
		static bool ni_update();
		static void ni_shutdown();

		bool initialize(char* uri = NULL);

		bool enable_depth_generator();
		bool enable_color_generator();
		bool enable_ir_generator();
		bool disable_depth_generator();
		bool disable_color_generator();
		bool disable_ir_generator();

		bool update();
		bool update_images();
		bool update_users();

		bool get_depth_16(cv::Mat &depth);
		bool get_depth_8(cv::Mat &depth);
		bool get_mask(cv::Mat &mask);
		bool get_color(cv::Mat &color);

		double get_frame_rate();

	private:
		//void init_variables();

	private:
		static bool					_ni2_initialized;
		static int					_ni2_n_streams;
		static openni::VideoStream**_ni2_g_streams;
		static int					_ni2_active_streams;

	private:
		int _flag_generators[3];

		openni::Device*		 _device;
		openni::VideoStream* _g_depth;
		openni::VideoStream* _g_color;
		openni::VideoStream* _g_ir;

		openni::VideoFrameRef* _frame_depth;
		openni::VideoFrameRef* _frame_color;
		openni::VideoFrameRef* _frame_ir;

		nite::UserTracker*			_nite_user_tracker;
		nite::UserTrackerFrameRef*	_nite_user_tracker_frame;
		nite::SkeletonState**		_nite_skeleton_states;
		

		////Processing
		//int _min_depth;
		//int _max_depth;

		//OpenCV Mat
		int _image_y;
		int _image_x;

		cv::Mat _depth_mat_16;
		cv::Mat _depth_mat_8;
		cv::Mat _color_mat;
		cv::Mat _mask_mat;

		//Frame Rate
		double	_last_tick;
		int		_frame_counter;
		double	_frame_rate;
};
#endif//_NI_KINECT_2