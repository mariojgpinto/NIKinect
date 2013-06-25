/** 
 * @file	NIThreadedKinect.h 
 * @author	Mario Pinto (mariojgpinto@gmail.com) 
 * @date	June, 2013 
 * @brief	Declaration of the NIThreadedKinect Class.
 */

#ifndef _NI_THREADED_KINECT
#define _NI_THREADED_KINECT

#include "NIKinect.h"

#include <boost\thread\mutex.hpp>
#include <boost\thread\thread.hpp>

#include <pcl\point_cloud.h>
#include <pcl\point_types.h>

/**
 * @class	NIThreadedKinect
 * @brief	Wrapper for OpenNI to use Kinect with threads.
 * @details	
 */
class __declspec(dllexport) NIThreadedKinect : public NIKinect{
	public:
		/**
		 * @brief	Possible Threads.
		 */
		enum THREAD_ID{
			CAPTURE_T = 0,			/**< */
			POINT_CLOUD_T = 1,		/**< */
		};
		static const int _n_threads = 2;

		enum THREAD_STATUS{
			NOT_ACTIVE,
			RUNNING,
			STOPPED
		};
		
		//Explanation: http://www.pcl-users.org/Eigen-Dense-Storage-Assertion-Error-Solved-td4023763.html
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	public:
		NIThreadedKinect();
		~NIThreadedKinect();
		
		//SETUP
		//void set_thread_status(THREADS thread, bool status);
		

		//control
		void start_thread(NIThreadedKinect::THREAD_ID id);
		bool is_running(NIThreadedKinect::THREAD_ID id);
		void stop_thread(NIThreadedKinect::THREAD_ID id);


		//ACCESS
		//bool get_depth(cv::Mat &depth);
		//bool get_mask(cv::Mat &mask);
		//bool get_color(cv::Mat &color);
		//bool get_depth_as_color(cv::Mat &depth_as_color);
		//
		//bool get_range_depth(cv::Mat &depth, int min = -1, int max = -1);
		//bool get_range_mask(cv::Mat &mask, int min = -1, int max = -1);
		//bool get_range_color(cv::Mat &color, int min = -1, int max = -1);
		//bool get_point_cloud();


		//MANUAL CONTROL
		void mutex_lock(NIThreadedKinect::THREAD_ID id);
		bool mutex_try_lock(NIThreadedKinect::THREAD_ID id);
		void mutex_unlock(NIThreadedKinect::THREAD_ID id);	

		XnPoint3D** get_3d_points();
		pcl::PointCloud<pcl::PointXYZ> * get_point_cloud();

		bool copy_3d_points(XnPoint3D *_point_3d);
		bool copy_point_cloud(pcl::PointCloud<pcl::PointXYZ> &cloud);
		
		

	private:
		//SETUP
		//void nikinect_(THREADS thread, bool status);
		void run_kinect();
		bool update_kinect();
		void run_point_cloud();
		bool update_point_cloud();

	private:
		//FLAGS
		/**
		 * @brief	Active Threads control.
		 * @details	
		 */
		bool _flags_threads[_n_threads];
		
		bool _running[_n_threads];
		boost::thread *_threads[_n_threads];
		boost::mutex *_mutex[_n_threads];

		//Point Cloud
		bool new_point_cloud;
		int _n_points;
		XnPoint3D *_point_2d;
		XnPoint3D *_point_3d;
		XnPoint3D *_point_3d_access;
		pcl::PointCloud<pcl::PointXYZ> _pcl_cloud;
		pcl::PointCloud<pcl::PointXYZ> _pcl_cloud_access;
		

		xn::DepthMetaData _depth_md_copy;
		cv::Mat _color_mat_copy;
};

#endif//_NI_THREADED_KINECT