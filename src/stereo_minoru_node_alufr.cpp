/* copied from ROS tutorial
 * "simple image subscriber"
 * */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv/cxcore.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "stereo_msgs/DisparityImage.h"
#include "cv_bridge/CvBridge.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include <face_detector/faces.h>
#include <image_geometry/stereo_camera_model.h>

using namespace std;
using namespace people;

class ViewerTest2 {

	public: 

	ros::NodeHandle nh;
	image_geometry::StereoCameraModel cam_model_;
		
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter limage_sub_; /**< Left image msg. */
	message_filters::Subscriber<stereo_msgs::DisparityImage> dimage_sub_; /**< Disparity image msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
	message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
	sensor_msgs::CvBridge lbridge_; /**< ROS->OpenCV bridge for the left image. */
	sensor_msgs::CvBridge dbridge_; /**< ROS->OpenCV bridge for the disparity image. */
	message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync_; /**< Stereo topic synchronizer. */

	image_transport::Publisher pub;
	Faces *detector;
	string name_; /**< Name of the detector. Ie frontalface, profileface. These will be the names in the published face location msgs. */
	string haar_filename_; /**< Training file for the haar cascade classifier. */
	double reliability_;



	/* callback method for synchronized stereo images
	 *
	 * */
	void ImageCallback(
		const sensor_msgs::Image::ConstPtr &limage,
		const stereo_msgs::DisparityImage::ConstPtr& dimage,
		const sensor_msgs::CameraInfo::ConstPtr& lcinfo,
		const sensor_msgs::CameraInfo::ConstPtr& rcinfo)
	{
	  ROS_INFO_STREAM_NAMED("Viewer Test", "got callback.");
	  sensor_msgs::CvBridge bridge_;
	  try
	  {
		cam_model_.fromCameraInfo(lcinfo, rcinfo);
		cv::Mat limageMat = bridge_.imgMsgToCv(limage, "bgr8");
		sensor_msgs::Image im = (*dimage).image;
		
		
		cv::Mat dimageMat = bridge_.imgMsgToCv(boost::make_shared<sensor_msgs::Image>(dimage->image), "bgr8");
		vector<Box2D3D> faces_vector = detector->detectAllFaces(limageMat, 1.0, dimageMat, &cam_model_);

		Box2D3D *one_face;

		for (uint iface = 0; iface < faces_vector.size(); iface++) {
		        one_face = &faces_vector[iface];
			ROS_INFO_STREAM_NAMED("Viewer Test", "face");

		}
	
	  }
	  catch (...)
	  {
	    ROS_ERROR("Error in ViewerTest2");
	  }
	}




	/* Constructor
	 *
	 * */
	ViewerTest2(std::string name) :
		it_(nh),
		sync_(4),
		detector(0)
	{
	  //~ ros::init(argc, argv, "image_listener");
	  ros::NodeHandle nh;
	  
	    detector = new Faces();
	    double face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m;
	    
	    // Parameters
	    //~ local_nh.param("classifier_name",name_,std::string(""));
	    //~ local_nh.param("classifier_filename",haar_filename_,std::string(""));
	    //~ local_nh.param("classifier_reliability",reliability_,0.0);
	    //~ local_nh.param("do_display",do_display_,std::string("none"));
	    //~ local_nh.param("do_continuous",do_continuous_,true);
	    //~ local_nh.param("do_publish_faces_of_unknown_size",do_publish_unknown_,false);
	    //~ local_nh.param("use_depth",use_depth_,true);
	    //~ local_nh.param("use_external_init",external_init_,true);
	    nh.param("face_size_min_m",face_size_min_m,Faces::FACE_SIZE_MIN_M);
	    nh.param("face_size_max_m",face_size_max_m,Faces::FACE_SIZE_MAX_M);
	    nh.param("max_face_z_m",max_face_z_m,Faces::MAX_FACE_Z_M);
	    nh.param("face_separation_dist_m",face_sep_dist_m,Faces::FACE_SEP_DIST_M);
	    
	    detector->initFaceDetection(1, "", face_size_min_m, face_size_max_m, max_face_z_m, face_sep_dist_m);
	
	    string stereo_namespace, image_topic;
	    stereo_namespace = nh.resolveName("stereo");

	    //subscribes to stereo cameras
	    string left_camera_info_topic = ros::names::clean(stereo_namespace + "/left/camera_info");
	    string right_camera_info_topic = ros::names::clean(stereo_namespace + "/right/camera_info");
	    limage_sub_.subscribe(it_,"/wide_stereo/left/image_color",3);
	    dimage_sub_.subscribe(nh,"/wide_stereo/disparity",3);
	    lcinfo_sub_.subscribe(nh,left_camera_info_topic,3);
	    rcinfo_sub_.subscribe(nh,right_camera_info_topic,3);    
	    sync_.connectInput(limage_sub_, dimage_sub_, lcinfo_sub_, rcinfo_sub_),
	    sync_.registerCallback(boost::bind(&ViewerTest2::ImageCallback, this, _1, _2, _3, _4));


	    //advertise topic
	    pub = it_.advertise("face_find/image", 1);

	    
	    ros::MultiThreadedSpinner s(2);
	    ros::spin(s);
	}

  
 	//destructor
	~ViewerTest2(){
	 
	}





};



int main(int argc, char **argv)
{
  ros::init(argc,argv,"viewertest");

  ViewerTest2 vt(ros::this_node::getName());

  return 0;
}

