/** 
 * @file	NIKinect.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	May, 2013 
 * @brief	Declaration of the NIKinect Class.
 */

#ifndef _NI_KINECT
#define _NI_KINECT

/** 
 * \mainpage NIKinect Index Page
 * 
 * \section intro_sec Introduction
 *
 * This is the introduction for NIKinect.
 *
 * \section install_sec Installation for NIKinect.
 *
 * \subsection step1 Step 1: 
 * 					 You open the box.
 *
 * \subsection step2 Step 2: 
 *					 Cut an hole in the box.
 *
 * \subsection step3 Step 3: 
 *					 Put your chunk in the box.
 *
 * \section conclusion Conclusion 
 *
 *	This is the conclusion of for NIKinect. See \ref install_sec for more details.
 *
 */

#include <XnCppWrapper.h>
#include <opencv2\opencv.hpp>

/**
 * @class	NIKinect
 * @brief	Wrapper for OpenNI to use Kinect.
 * @details	
 */
class __declspec(dllexport) NIKinect{
	public:
		/**
		 * @brief	Generation Flags.
		 */
		enum FLAGS{
			DEPTH_G = 1,	/**< Controls the generatation of Depth MetaData information. */
			IMAGE_G = 2,	/**< Controls the generatation of Image MetaData information. */
			IR_G = 4,		/**< Controls the generatation of IR    MetaData information. */
			AUDIO_G = 8,	/**< Controls the generatation of Audio MetaData information. */
			USER_G = 16,	/**< */
			GESTURE_G = 32,	/**< */
			HAND_G = 64,	/**< */
			SCENE_A = 128,	/**< */
			
		};
		/**< Number of Generation Flags */
		static const int _n_flags = 8; 

		/**
		 * @brief	Processing Flags.
		 */
		enum PROCESSING{
			DEPTH_P = 1,		/**< */
			MASK_P = 2,			/**< */
			IMAGE_P  = 4,		/**< */
			DEPTH_COLOR = 8,	/**< */
			POINT_CLOUD = 16,	/**< */
			POINT_CLOUD_PCL = 32,	/**< */
		};
		/**< Number of Processing Flags */
		static const int _n_processing = 6;

	public:
		NIKinect();
		~NIKinect();

		//Setup
		bool init(const char* file = 0, int generators = DEPTH_G + IMAGE_G);
		void set_min_depth(int milimeters);
		void set_max_depth(int milimeters);
		void set_processing_flag(NIKinect::PROCESSING flag, bool value);

		//Running
		virtual bool update();

		//Processing
		bool get_floor_plane(double *a, double *b, double *c, double *d);
		//void remove_plane(cv::Mat& mask, double a, double b, double c, double d);

		//3D
		bool convert_to_realworld(int count, XnPoint3D* points_in, XnPoint3D* points_out);
		bool convert_to_realworld(cv::Mat mask, XnPoint3D* points_out, int min_x = -1, int max_x = -1, int min_y = -1, int max_y = -1);
		//bool convert_to_realworld(cv::Mat mask, XnPoint3D* points_out);

		void set_3d_analysis_step(int step);

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

		//Access Meta Data
		bool get_depth_meta_data(xn::DepthMetaData& depth);

		//Access - Convert to OpenCV
		virtual bool get_depth(cv::Mat &depth);
		virtual bool get_mask(cv::Mat &mask);
		virtual bool get_color(cv::Mat &color);
		//bool get_depth_as_color(cv::Mat &depth_as_color, int min = -1, int max = -1);
		virtual bool get_depth_as_color(cv::Mat &depth_as_color);

		virtual bool get_range_depth(cv::Mat &depth, int min = -1, int max = -1);
		virtual bool get_range_mask(cv::Mat &mask, int min = -1, int max = -1);
		//virtual bool get_range_color(cv::Mat &color, int min = -1, int max = -1);
		//bool get_range_depth_as_color(cv::Mat &depth_as_color, int min = -1, int max = -1);

		//Access 3D
		XnPoint3D* get_points_3d();
		int get_3d_analysis_step();
		
		double get_frame_rate();

		static void compute_color_encoded_depth(const cv::Mat1f& depth_im, cv::Mat& color_depth_im,
                                     double* i_min_val, double* i_max_val);

	private:
		bool init_from_xml_file(const char* file = 0);
		void flag_re_check();

		bool init_depth_generator();
		bool init_image_generator();
		bool init_ir_generator();
		bool init_audio_generator();
		
		bool init_user_generator();
		bool init_gesture_generator();
		bool init_hand_generator();
		
		bool init_scene_analyzer();

		virtual void generate_point_cloud();

	protected:
		void update_frame_rate();
		
	protected:
		//Flags
		/**
		 * @brief Generation Flags. 
		 * @details Generation Flags Details.
		 */
		bool _flags[_n_flags * _n_flags + _n_flags + 1];
		bool _flags_processing[_n_processing * _n_processing + _n_processing + 1];
		
		//Context
		xn::Context _context;
		xn::ScriptNode _scriptNode;

		//Generators
		xn::DepthGenerator _depth_generator;		/**< OpenNI's Depth		Generator.*/
		xn::ImageGenerator _image_generator;		/**< OpenNI's Image		Generator.*/
		xn::IRGenerator _ir_generator;				/**< OpenNI's IR		Generator.*/
		xn::AudioGenerator _audio_generator;		/**< OpenNI's Audio		Generator.*/

		xn::UserGenerator _user_generator;			/**< OpenNI's User		Generator.*/
		xn::GestureGenerator _gesture_generator;	/**< OpenNI's Gesture	Generator.*/
		xn::HandsGenerator _hands_generator;		/**< OpenNI's Hand		Generator.*/
		
		xn::SceneAnalyzer _scene_analyzer;			/**< OpenNI's Scene		Analyser .*/

		//Meta Data
		xn::DepthMetaData _depth_md;	/**< OpenNI's Depth	MetaData.*/
		xn::ImageMetaData _image_md;	/**< OpenNI's Image	MetaData.*/
		xn::IRMetaData _ir_md;			/**< OpenNI's IR	MetaData.*/
		xn::AudioMetaData _audio_md;	/**< OpenNI's Audio	MetaData.*/
		xn::SceneMetaData _scene_md;	/**< OpenNI's Scene	MetaData.*/

		//Processing
		int _min_depth;
		int _max_depth;

		//OpenCV Mat
		cv::Mat _depth_mat;
		cv::Mat _color_mat;
		cv::Mat _mask_mat;
		cv::Mat _depth_as_color_mat;

		//Real World Coordinates?
		int _point_step;
		XnPoint3D * _point_2d;
		XnPoint3D * _point_3d;
		//pcl::PointCloud<pcl::PointXYZ> _cloud_pcl;

		//Frame Rate
		double _last_tick;
		int _frame_counter;
		double _frame_rate;

		static const char* _sample_xml_path;
};

#endif //_NI_KINECT