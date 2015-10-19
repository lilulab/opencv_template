#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv.h>
#include <highgui.h>

using namespace cv;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;


static const std::string OPENCV_WINDOW = "Image window";

cv::Mat output,overlay;
Mat temp, color_temp; //setup some temps

//cv_bridge::CvImagePtr cv_ptr;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam_back/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/cam_calib/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

	void ImageProcess(const cv::Mat& src, cv::Mat& out) {

	  src.copyTo(out);

	  if (src.rows > 60 && src.cols > 60) {

	  	// Center circle
	  	cv::circle(output, cv::Point(src.cols/2,src.rows/2), 10, CV_RGB(0,0,255));
	  	cv::circle(output, cv::Point(src.cols/2,src.rows/2), 5, CV_RGB(255,0,0));

	  	// Cross hair - vertical
        cv::line( output, Point(src.cols/2,0),Point(src.cols/2,src.rows/2-10), Scalar(255,0,255), 1, 8 ); 
        cv::line( output, Point(src.cols/2,src.rows/2+10),Point(src.cols/2,src.rows), Scalar(255,0,255), 1, 8 ); 

        // Cross hair - horizontal
        cv::line( output, Point(0,src.rows/2),Point(src.cols/2-10,src.rows/2), Scalar(255,0,255), 1, 8 ); 
        cv::line( output, Point(src.cols/2+10,src.rows/2),Point(src.cols,src.rows/2), Scalar(255,0,255), 1, 8 ); 
	  }

	}


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;

    cv_bridge::CvImage out_msg;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    ImageProcess(cv_ptr->image,output);

    // Update GUI Window
	cv::imshow(OPENCV_WINDOW, output);

    cv::waitKey(3);
    
    // Output modified video stream
	out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
	out_msg.image    = output; // Your cv::Mat

 	image_pub_.publish(out_msg.toImageMsg());
	//saliency_img_pub.publish(out_msg.toImageMsg());
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_calib");
  ImageConverter ic;
  ros::spin();
  return 0;
}
