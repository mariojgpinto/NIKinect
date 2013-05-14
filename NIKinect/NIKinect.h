/** 
 * @file NIKinect.h 
 * @author Mario Pinto (mario.pinto@ccg.pt) 
 * @date May, 2013 
 * @brief Declaration of the NIKinect Class.
 */
#ifndef _NI_KINECT
#define _NI_KINECT

#include <XnCppWrapper.h>
#include <opencv2\opencv.hpp>

/**
 * @class	NIKinect
 * @brief	Wrapper for OpenNI to use Kinect.
 * @details	.
 */
class __declspec(dllexport) NIKinect{
	public:
		enum FLAGS{
			DEPTH_G = 1,
			IMAGE_G = 2,
			IR_G = 4,
			AUDIO_G = 8,
			USER_G = 16,
			GESTURE_G = 32,
			HAND_G = 64,
			SCENE_A = 128
		};
		static const int _n_flags = 8;

	public:
		NIKinect();
		~NIKinect();

		//Setup
		bool init(const char* file = 0, int generators = DEPTH_G + IMAGE_G + SCENE_A);
		void set_min_depth(int milimeters);
		void set_max_depth(int milimeters);

		//Running
		bool update();

		//Processing
		bool get_floor_plane(double *a, double *b, double *c, double *d);
		//void remove_plane(cv::Mat& mask, double a, double b, double c, double d);


		//Access
		bool check_flag(NIKinect::FLAGS flag);
	
		int get_min_depth();
		int get_max_depth();

		//Access - Generator
		xn::Context& get_context();
		xn::DepthGenerator& get_depth_generator();
		xn::ImageGenerator& get_image_generator();
		xn::IRGenerator& get_ir_generator();
		xn::AudioGenerator& get_audio_generator();

		xn::UserGenerator& get_user_generator();
		xn::GestureGenerator& get_gesture_generator();
		xn::HandsGenerator& get_hands_generator();

		xn::SceneAnalyzer& get_scene_analyzer();

		//Access - Convert to OpenCV
		bool get_depth(cv::Mat &depth);
		bool get_mask(cv::Mat &mask);
		bool get_color(cv::Mat &color);
		bool get_depth_as_color(cv::Mat &depth_as_color, int min = -1, int max = -1);

		bool get_range_depth(cv::Mat &depth, int min = -1, int max = -1);
		bool get_range_mask(cv::Mat &mask, int min = -1, int max = -1);
		bool get_range_color(cv::Mat &color, int min = -1, int max = -1);
		//bool get_range_depth_as_color(cv::Mat &depth_as_color, int min = -1, int max = -1);

		bool get_depth_meta_data(xn::DepthMetaData *depth);


		double get_frame_rate();

	private:
		bool init_from_xml_file(const char* file = 0);
		void flag_re_check();

		//void init_from_xml_file(const char* file = 0);
		bool init_depth_generator();
		bool init_image_generator();
		bool init_ir_generator();
		bool init_audio_generator();
		
		bool init_user_generator();
		bool init_gesture_generator();
		bool init_hand_generator();
		
		bool init_scene_analyzer();

		void update_frame_rate();
		static void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat& color_depth_im,
                                     double* i_min_val, double* i_max_val);
		
	private:
		//Flags
		bool _flags[_n_flags * _n_flags + _n_flags + 1];

		//Context
		xn::Context _context;
		xn::ScriptNode _scriptNode;

		//Generators
		xn::DepthGenerator _depth_generator;
		xn::ImageGenerator _image_generator;
		xn::IRGenerator _ir_generator;
		xn::AudioGenerator _audio_generator;

		xn::UserGenerator _user_generator;
		xn::GestureGenerator _gesture_generator;
		xn::HandsGenerator _hands_generator;
		
		xn::SceneAnalyzer _scene_analyzer;

		//Meta Data
		xn::DepthMetaData _depth_md;
		xn::ImageMetaData _image_md;
		xn::IRMetaData _ir_md;
		xn::AudioMetaData _audio_md;

		//Processing
		int _min_depth;
		int _max_depth;

		//OpenCV Mat
		cv::Mat _depth_mat;
		cv::Mat _color_mat;
		cv::Mat _mask_mat;


		//Frame Rate
		double _last_tick;
		int _frame_counter;
		double _frame_rate;

		static const char* _sample_xml_path;
};

#endif //_NI_KINECT