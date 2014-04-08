
#include <NIKinect.h>
#include <NIKinect2.h>
//#include <NIThreadedKinect.h>

int main_ni_threaded_kinect(int argc, char* argv[]);
int main_ni_kinect(int argc, char* argv[]);
int main_multi_ni_kinect(int argc, char* argv[]);
int main_ni_kinect2(int argc, char* argv[]);

int main(int argc, char* argv[]){
	//main_ni_threaded_kinect(argc, argv);
	//main_ni_kinect(argc, argv);
	//main_multi_ni_kinect(argc, argv);
	main_ni_kinect2(argc, argv);

	return 0;
}


int main_ni_kinect2(int argc, char* argv[]){

	//{
	//	openni::Device device;        // Software object for the physical device i.e. 
 //                                         // PrimeSense Device Class
 //       openni::VideoStream ir;       // IR VideoStream Class Object
 //       openni::VideoFrameRef irf;    //IR VideoFrame Class Object
 //       openni::VideoMode vmode;      // VideoMode Object
 //       openni::Status rc = openni::STATUS_OK;

 //       rc = openni::OpenNI::initialize();    // Initialize OpenNI
 //       rc = device.open(openni::ANY_DEVICE); // Open the Device
 //       rc = ir.create(device, openni::SENSOR_IR);    // Create the VideoStream for IR
 //       rc = ir.start();                      // Start the IR VideoStream

 //       cv::Mat frame, raw;                                // OpenCV Matrix Object, also used to store images
 //       int h, w;                                // Height and Width of the IR VideoFrame

 //       while(true)                                // Crux of this project
 //       {
 //               if(device.getSensorInfo(openni::SENSOR_IR) != NULL)
 //               {
 //                       rc = ir.readFrame(&irf);                // Read one IR VideoFrame at a time
 //                       if(irf.isValid())                                // If the IR VideoFrame is valid
 //                       {
 //                               vmode = ir.getVideoMode();  // Get the IR VideoMode Info for this video stream. 
 //                               
	//							
	//							// This includes its resolution, fps and stream format.
	//							cv::Mat ir_raw(irf.getHeight(),irf.getWidth(),CV_8UC1,(void*)irf.getData());
	//							////ir_raw.convertTo(frame, CV_8UC1);
 //       //                        const uchar* imgBuf = (const uchar*)irf.getData(); 
 //       //                                                                            // PrimeSense gives the IR stream as 16-bit data output
 //       //                        h=irf.getHeight(); 
 //       //                        w=irf.getWidth();
	//							//raw.create(h, w, CV_16UC1); // Create the OpenCV Mat Matrix Class Object 
 //       //                                                                                // to receive the IR VideoFrames
 //       //                        memcpy(raw.data, imgBuf, h*w*sizeof(uchar)); 
 //                                                                                       // Copy the ir data from memory imgbuf -> frame.data 
 //                                                                                       // using memcpy (a string.h) function
 //                               //raw.convertTo(frame, CV_8UC1); 
 //                                                                                       // OpenCV displays 8-bit data (I'm not sure why?) 
 //                                                                                       // So, convert from 16-bit to 8-bit
 //                               //namedWindow("ir", 1);                // Create a named window
 //                               cv::imshow("ir", ir_raw);                // Show the IR VideoFrame in this window
 //                               char key = cv::waitKey(10);
 //                               if(key==27) break;                        // Escape key number
 //                       }
 //               }
 //       }
 //       //--------Safe closing--------//
 //       ir.stop();                                                                // Stop the IR VideoStream
 //       ir.destroy();
 //       device.close();                                                        // Close the PrimeSense Device
	//}
	//return 0;


	int hd = 1;

	start:
	
	if(!NIKinect2::ni_initialize())
		return false;

	NIKinect2* kinect = new NIKinect2();

	bool result = false;

	if(argc == 2){
		result = kinect->initialize(argv[1]);
	}
	else{
		result = kinect->initialize();
	}

	if(result){
		
		
		kinect->enable_color_generator();
		//kinect->enable_depth_generator();
		//kinect->enable_ir_generator();
		//kinect->enable_user_generator();
		//kinect->enable_hand_tracker();

		//kinect->set_color_hd(hd);

		//kinect->set_depth_color_registration(true);

		
	}	
	else{
		return false;
	}

	cv::Mat color;
	cv::Mat depth;
	cv::Mat ir;

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!NIKinect2::ni_update())
			break;
		//printf("Up..");
		if(!kinect->update())
			break;
		//printf(" .dated");

		if(kinect->get_color(color)){
			//cv::circle(color,cv::Point((int)kinect->xx[0],(int)kinect->yy[0]),5,cv::Scalar(0,0,255),-1);
			//cv::circle(color,cv::Point((int)kinect->xx[1],(int)kinect->yy[1]),5,cv::Scalar(0,255,0),-1);
			imshow("Color",color);
		}
		//printf(" color.. ");
		if(kinect->get_depth_8(depth)){
			imshow("Depth",depth);
		}
		//printf("depth.. ");

		if(kinect->get_ir(ir)){
			imshow("IR",ir);
		}
		//printf("ir.. ");

		//printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
		if(c == ' '){
			kinect->~NIKinect2();
			NIKinect2::ni_shutdown();
			break;
		}

		//printf("END\n");
	}

	kinect->~NIKinect2();
	NIKinect2::ni_shutdown();

	goto start;

	return 0;
}

#include <_ID.h>

//#include "multi_NIKinect.h"

XnCallbackHandle _openni_callback_user_handle;

void XN_CALLBACK_TYPE openni_callback_user_new(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	//UserManager *instance = (UserManager*)pCookie;
	printf("New User\n");
	//instance->add_openni_event(nId,UserManager::NEW_USER);
}

void XN_CALLBACK_TYPE openni_callback_user_lost(xn::UserGenerator& generator, XnUserID nId, void* pCookie){
	//UserManager *instance = (UserManager*)pCookie;
	printf("User Lost\n");
	//instance->add_openni_event(nId,UserManager::LOST_USER);
}



int main_multi_ni_kinect(int argc, char* argv[]){
	NIKinect* kinect1 = new NIKinect();
	NIKinect* kinect2 = new NIKinect();
	bool result = false;


#ifdef _CCG
	result = kinect1->init(0,0,3+NIKinect::SCENE_A);//,3+NIKinect::SCENE_A);//);	
	//result = kinect2->init(0,0,3+NIKinect::SCENE_A);//,3+NIKinect::USER_G);//,3+NIKinect::SCENE_A);//
	

#endif

#ifdef _HOME
	result = kinect1->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni", NIKinect::DEPTH_G + NIKinect::IMAGE_G + NIKinect::SCENE_A + NIKinect::USER_G);
#endif

	//xn::UserGenerator _user_generator = kinect1 ->get_user_generator();

	//XnStatus s = _user_generator.RegisterUserCallbacks(	openni_callback_user_new,
	//										openni_callback_user_lost,
	//										NULL,
	//										_openni_callback_user_handle);

	//xn::SceneAnalyzer xn_scene1;
	//xn::SceneAnalyzer xn_scene2;

	cv::Mat color_1;
	cv::Mat color_2;
	cv::Mat depth_1;
	cv::Mat depth_2;
	cv::Mat mask_1;
	cv::Mat mask_2;
	cv::Mat depthMat8UC1_1;
	cv::Mat depthMat8UC1_2;
	cv::Mat3b depth_as_color;

	int _min_bar = 400;
	int _max_bar = 1500;

	//cv::namedWindow("DepthAsColor");
	//cv::createTrackbar("MinDepth", "DepthAsColor", &_min_bar, 5000, NULL);
	//cv::createTrackbar("MaxDepth", "DepthAsColor", &_max_bar, 5000, NULL);

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect1->update()) 
			break;
		if(kinect1->get_color(color_1)){
			kinect1->get_mask(mask_1);
			cv::Mat color;

			color_1.copyTo(color,mask_1);
			imshow("Color1",color);
		}
		//if(kinect1->get_depth(depth_1)){
		//	depth_1.convertTo(depthMat8UC1_1, CV_8UC1,0.05);
		//	imshow("Depth1",depthMat8UC1_1);
		//}



		//if(!kinect2->update()) 
		//	break;
		//if(kinect2->get_color(color_2)){
		//	kinect2->get_mask(mask_2);
		//	cv::Mat color;

		//	color_2.copyTo(color,mask_2);
		//	imshow("Color2",color);
		//}
		//if(kinect2->get_depth(depth_2)){
		//	depth_2.convertTo(depthMat8UC1_2, CV_8UC1,0.05);
		//	imshow("Depth2",depthMat8UC1_2);
		//}

		

		if(c == '1'){
			double a,b,c,d;

			if(kinect1->get_floor_plane(&a,&b,&c,&d)){
				printf("Floor 1 (%.4f  %.4f  %.4f  %.4f)\n",a,b,c,d);
			}
			else{
				printf("No Floor 1\n");
			}
			//XnPlane3D floorCoords;
			//if(XN_STATUS_OK == kinect1->get_scene_analyzer().GetFloor(floorCoords)){
			//	printf("Floor 1\n");
			//}
		}

		if(c == '2'){
			double a,b,c,d;

			if(kinect2->get_floor_plane(&a,&b,&c,&d)){
				printf("Floor 2 (%.4f  %.4f  %.4f  %.4f)\n",a,b,c,d);
			}
			else{
				printf("No Floor 2\n");
			}
			//XnPlane3D floorCoords;
			//if(XN_STATUS_OK == kinect2->get_scene_analyzer().GetFloor(floorCoords)){
			//	printf("Floor 2\n");
			//}
		}

		//printf("Kinect1 (%.2f)    Kinect2 (%.2f)\n",kinect1->get_frame_rate(), kinect2->get_frame_rate());
	}


	return 0;
}

int main_ni_kinect(int argc, char* argv[]){
	NIKinect* kinect = new NIKinect();

	if(argc == 2){
		kinect->init(argv[1]);
	}
	else{
		kinect->init(0,0);
	}

	//kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	//kinect->init("C:\\Dev\\Walkys\\Project\\Data\\foot_2_front.oni");
	//kinect->init();

	bool result = false;

//	result = kinect->init_generators();

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat3b depth_as_color;

	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		if(!kinect->update()) 
			break;

		if(kinect->get_color(color)){
			imshow("Color",color);
		}
		
		if(kinect->get_depth(depth)){
			depth.convertTo(depthMat8UC1, CV_8UC1,0.05);
			imshow("Depth",depthMat8UC1);

			if(c == ' '){
				static int n = 1;
				char buff[128];
				sprintf(buff,"img_%02d.png",n++);
				cv::imwrite(buff,depthMat8UC1);
			}
			//
			//kinect->get_mask(mask);
			//imshow("Mask",mask);
		
			if(kinect->get_depth_as_color(depth_as_color)){
				//kinect->get_depth_as_color(depth_as_color);//,500,2500);
				cv::imshow("DepthAsColor",depth_as_color);
			}
			//cv::Mat masked_color;
			//color.copyTo(masked_color,mask);
			//imshow("MaskedColor",masked_color);
		}

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
	}


	return 0;
}
/*
int main_ni_threaded_kinect(int argc, char* argv[]){
	NIThreadedKinect* kinect = new NIThreadedKinect();

	if(argc == 2){
		kinect->init(argv[1]);
	}
	else{
		kinect->init();
	}

	//kinect->init("C:\\Dev\\Kinect\\Data\\ONI\\mirror_papers.oni");
	//kinect->init("C:\\Dev\\Walkys\\Project\\Data\\foot_2_front.oni");
	//kinect->init();

	kinect->start_thread(NIThreadedKinect::CAPTURE_T);
	Sleep(1000);
	kinect->start_thread(NIThreadedKinect::POINT_CLOUD_T);

	bool result = false;

//	result = kinect->init_generators();

	cv::Mat color;
	cv::Mat depth;
	cv::Mat mask;
	cv::Mat depthMat8UC1;
	cv::Mat3b depth_as_color;

	
	
	char c = 0;
	while((c = cv::waitKey(31)) != 27){
		//if(!kinect->update()) 
		//	break;

		kinect->mutex_lock(NIThreadedKinect::CAPTURE_T);
		if(kinect->get_color(color)){
			imshow("Color",color);
		}

		

		if(kinect->get_depth(depth)){
			depth.convertTo(depthMat8UC1, CV_8UC1,0.05);
			imshow("Depth",depthMat8UC1);

			if(c == ' '){
				static int n = 1;
				char buff[128];
				sprintf(buff,"img_%02d.png",n++);
				cv::imwrite(buff,depthMat8UC1);
			}
			//
			//kinect->get_mask(mask);
			//imshow("Mask",mask);
		
			if(kinect->get_depth_as_color(depth_as_color)){
				//kinect->get_depth_as_color(depth_as_color);//,500,2500);
				cv::imshow("DepthAsColor",depth_as_color);
			}
			//cv::Mat masked_color;
			//color.copyTo(masked_color,mask);
			//imshow("MaskedColor",masked_color);
		}

		kinect->mutex_unlock(NIThreadedKinect::CAPTURE_T);

		printf("Frame Rate: %.2f\n",kinect->get_frame_rate());
	}


	return 0;
}
*/