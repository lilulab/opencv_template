#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <math.h>       /* atan2 */
#define PI 3.14159265

#include <cv.h>
#include <highgui.h>

#include "crawler_msgs/VisualHeading.h"

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
  ros::Publisher msg_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    msg_pub_ = nh_.advertise<crawler_msgs::VisualHeading>("crawler/visual_heading", 1);

    cv::namedWindow(OPENCV_WINDOW);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

	float getHeading(const cv::Mat& src, cv::Mat& out) {
	  Mat temp, color_temp; //setup some temps
	  cvtColor(src, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
	  //Sobel(temp, temp, CV_8U, 1, 1);

	  //Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
	  //Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
		Canny( temp, temp, 50, 100, 3 ); //Works Nice        //Canny( temp, temp, 30, 90, 3 ); //run Canny edge detector with some default values
	  //Canny( temp, temp, lowThreshold, lowThreshold*ratio, kernel_size );
	  cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

	  vector<Vec4i> lines;  // Hough lines result

	  //vector<Vec3i> heading;  //robot heading vector
	  // int heading_average[2];
	  float heading[3];
	  float heading_average = 0;

	  HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
	  size_t i;

	  src.copyTo(temp);

	  for(i = 0; i < lines.size(); i++ )
	  {
		//line( color_temp, Point(lines[i][0], lines[i][1]),
		//    Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
		//ROS_INFO("Edge %d: P1(%d,%d) to P2(%d,%d).",i,lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
		line( temp, Point(lines[i][0], lines[i][1]),
		Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw

		heading[0] = (float)(lines[i][0] - lines[i][2]);
		heading[1]= (float)(lines[i][1] - lines[i][3]);


		heading[3] = atan2(heading[0],heading[1]); //Yaw angle in radian
		//heading[3] = -heading[3] - 90;  //correct orientation. 

		ROS_INFO("heading = [%f,%f]. atan2 = %f.",heading[0],heading[1],heading[3]);

		heading_average += heading[3]; // get sum

		// calulate heading vector using line gradiant
		//heading[i][0] = lines[i][0] - lines[i][1];
		//heading[i][1] = lines[i][2] - lines[i][3];

		//heading_average[0] += heading[i][0];
		//heading_average[1] += heading[i][1];
	  }

	  //heading_average[0] = heading_average[0] / lines.size();
	  //heading_average[1] = heading_average[1] / lines.size();

	  	heading_average = heading_average/(float)lines.size(); // Calculate average

	  	ROS_INFO("Heading Average = %f(radian)",heading_average);

		ROS_INFO("Found %d Edges!",i);
		//ROS_INFO("Found %d Edges! Heading: [%d,%d]", i,heading_average[0],heading_average[1]);
	  	out = color_temp;
		addWeighted( temp, 0.5, color_temp, 0.5, 0.0, overlay);
		//overlay = temp;

		return heading_average;
	}
/*

	void CannyThreshold(int, void*){
	  Mat temp, color_temp; //setup some temps
	  cvtColor(cv_ptr->image, temp, CV_BGR2GRAY); //convert to grayscale for the edge detector
	  //Sobel(temp, temp, CV_8U, 1, 1);

	  //Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
	  //Canny( temp, temp, 50, 200, 3 ); //run Canny edge detector with some default values
	  Canny( temp, temp, lowThreshold, lowThreshold*ratio, kernel_size );
	  cvtColor( temp, color_temp, CV_GRAY2BGR ); //Convert Canny edges back to 3-channel

	  vector<Vec4i> lines;
	  HoughLinesP( temp, lines, 1, CV_PI/180, 80, 30, 10 ); //Find lines in the Canny image
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
		line( color_temp, Point(lines[i][0], lines[i][1]),
		    Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 ); //Draw
	  }
		
	}
*/

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;

    //cv_bridge::CvImage out_msg;
    crawler_msgs::VisualHeading visual_heading;

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
      //cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 30, CV_RGB(0,0,255));
	  //cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2,cv_ptr->image.rows/2), 5, CV_RGB(255,0,0));
	  //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 100), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 500), 10, CV_RGB(255,0,0));
		//cv::circle(cv_ptr->image, cv::Point(10, 1000), 10, CV_RGB(255,0,0));
	//image.row = 960, image.cols = 1280 .
        //ROS_INFO("image.row = %d, image.cols = %d .",cv_ptr->image.rows,cv_ptr->image.cols);

    
    visual_heading.RPY_radian.z = getHeading(cv_ptr->image,output);

	//cv::imshow(OPENCV_WINDOW, overlay);

    //cv::waitKey(3);
    
    // Output modified video stream
	visual_heading.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    
    visual_heading.RPY_degree.z = visual_heading.RPY_radian.z * 180 / PI;

    visual_heading.RPY_degree.z = -visual_heading.RPY_degree.z -90;

 	msg_pub_.publish(visual_heading);
	//saliency_img_pub.publish(out_msg.toImageMsg());
    //msg_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "edge_detection");
  ImageConverter ic;
  ros::spin();
  return 0;
}
