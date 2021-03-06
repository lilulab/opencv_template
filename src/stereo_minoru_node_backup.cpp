#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW_LEFT = "Left Image";
static const std::string OPENCV_WINDOW_RIGHT = "Right Image";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_left_;
  image_transport::ImageTransport it_right_;
  image_transport::Subscriber image_left_sub_;
  image_transport::Subscriber image_right_sub_;
  //image_transport::Publisher image_left_pub_;
  
public:
  ImageConverter()
    : it_left_(nh_),it_right_(nh_) 
  {
    // Subscrive to input video feed and publish output video feed
    image_left_sub_ = it_left_.subscribe("/stereo/left/camera/image_raw", 1, &ImageConverter::imageCbLeft, this);
    image_right_sub_ = it_right_.subscribe("/stereo/right/camera/image_raw", 1, &ImageConverter::imageCbRight, this);
    //image_left_pub_ = it_left_.advertise("/opencv_proc/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW_LEFT);
    cv::namedWindow(OPENCV_WINDOW_RIGHT);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW_LEFT);    
    cv::destroyWindow(OPENCV_WINDOW_RIGHT);
  }

  void imageCbLeft(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 30, CV_RGB(255,0,0));

      //Do Something Cooler Here!

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_LEFT, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_left_pub_.publish(cv_ptr->toImageMsg());
  }


  void imageCbRight(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 30, CV_RGB(0,0,255));

      //Do Something Cooler Here!

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_RIGHT, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_right_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
