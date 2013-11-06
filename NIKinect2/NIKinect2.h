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
	
//class NIKinect2Manager;

/**
 * @class NIKinect2
 * @brief	Wrapper for OpenNI2 to use Kinect.
 * @details	NIKinect2.
 *
 * @TODO	- Sync size between HD Color and Depth
 *			- Add Floor calc and plane retrieve
 *			- Add Depth Control
 *			- Add Image Size Control (Update Generators)
 *			- Add 3D Convertion
 *			- Add access to Generators
 *			- Add Color Depth
 *			- Add ID
 **/
class __declspec(dllexport)NIKinect2{
	friend class NIKinect2Manager;

	public:
		enum GENERATORS{
			NI2_G_DEPTH = 0,
			NI2_G_COLOR = 1,
			NI2_G_IR = 2,
			NI2_G_USER = 3
		};
		static const int _n_generators = 4;

		static const int _max_users = 16;

	public:
		NIKinect2();
		~NIKinect2();

		static bool ni_initialize();
		static bool ni_update();
		static void ni_shutdown();

		bool initialize(char* uri = NULL);

		bool set_depth_color_registration(bool enable);

		bool set_color_hd(bool enable);

		bool enable_depth_generator();
		bool enable_color_generator();
		bool enable_ir_generator();
		bool enable_user_generator();

		bool disable_depth_generator();
		bool disable_color_generator();
		bool disable_ir_generator();
		bool disable_user_generator();

		bool update();
		bool update_images();
		bool update_nite();

		bool get_depth_16(cv::Mat &depth);
		bool get_depth_8(cv::Mat &depth);
		bool get_mask(cv::Mat &mask);
		bool get_color(cv::Mat &color);

		//NITE
		bool get_users_map(nite::UserMap &map);
		bool get_users_map(cv::Mat &map);

		std::vector<nite::UserData*>* get_users_data();
		std::vector<int>* get_users_ids();
		bool get_user_data(int idx, nite::UserData& data);
		bool get_user_mask(int idx, cv::Mat &mask);


		double get_frame_rate();

	public:
		//OpenNI Direct Access
		openni::Device*			get_device();
		openni::VideoStream*	get_depth_stream();
		openni::VideoStream*	get_color_stream();
		openni::VideoStream*	get_ir_stream();

		openni::VideoFrameRef*	get_depth_frame_ref();
		openni::VideoFrameRef*	get_color_frame_ref();
		openni::VideoFrameRef*	get_ir_frame_ref();

		nite::UserTracker*			get_user_tracker();
		nite::UserTrackerFrameRef*	get_user_tracker_frame();

	private:
		//void init_variables();

	private:
		static bool					_ni2_initialized;
		static int					_ni2_n_streams;
		static openni::VideoStream**_ni2_g_streams;
		static int					_ni2_active_streams;

	private:
		bool _flag_generators[NIKinect2::_n_generators];

		//std::string* _uri;

		openni::Device*		 _device;
		openni::VideoStream* _g_depth;
		openni::VideoStream* _g_color;
		openni::VideoStream* _g_ir;

		openni::VideoFrameRef* _frame_depth;
		openni::VideoFrameRef* _frame_color;
		openni::VideoFrameRef* _frame_ir;

		nite::UserTracker*				_nite_user_tracker;
		nite::UserTrackerFrameRef*		_nite_user_tracker_frame;
		nite::SkeletonState*			_nite_skeleton_states;
		nite::UserMap					_nite_user_map;
		std::vector<nite::UserData*>*	_users;
		std::vector<int>*				_users_ids;
		

		////Processing
		//int _min_depth;
		//int _max_depth;

		//OpenCV Mat
		openni::VideoMode _mode_depth;
		openni::VideoMode _mode_color;

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