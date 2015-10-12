#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;
using namespace message_filters;

void callback(const ImageConstPtr& image_left, const ImageConstPtr& image_right)
{
  // Solve all of perception here...
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_node");

  ros::NodeHandle nh;

  message_filters::Subscriber<Image> image_left_sub(nh, "/stereo/left/camera/image_raw", 1);
  message_filters::Subscriber<Image> image_right_sub(nh, "/stereo/right/camera/image_raw", 1);

  TimeSynchronizer<Image, Image> sync(image_left_sub, image_right_sub, 1);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
