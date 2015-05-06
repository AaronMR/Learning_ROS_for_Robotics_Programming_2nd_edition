
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

class CameraDriver
{
public:
  enum Default
  {
    DEFAULT_CAMERA_INDEX = 0,
    DEFAULT_FPS = 30
  };

  CameraDriver( int camera_index = DEFAULT_CAMERA_INDEX )
    : nh( "~" )
    , it( nh )
    , camera_index(camera_index)
    , camera( camera_index )
    , last( 0 )
  {
    nh.param<int>( "camera_index", camera_index, DEFAULT_CAMERA_INDEX );
    nh.param<int>( "fps", fps, DEFAULT_FPS );

    if ( not camera.isOpened() )
    {
      ROS_ERROR_STREAM( "Failed to open camera device!" );
      ros::shutdown();
    }

    period = ros::Duration( 1. / fps );

    pub_image_raw = it.advertise( "image_raw", 1 );

    frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;
  }

  ~CameraDriver()
  {
    camera.release();
  }

  void capture()
  {
    camera >> frame->image;	// blocking, so it frees the CPU.
    if ( frame->image.empty() )
    {
      ROS_ERROR_STREAM( "Failed to retrive new frame! " );
      ros::shutdown();
    }

    ros::Time now = ros::Time::now();
    if ( ( now - last ) >= period )
    {
      frame->header.stamp = now;
      pub_image_raw.publish( frame->toImageMsg() );

      last = now;
    }
  }

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pub_image_raw;

  int camera_index;
  int fps;

  cv::VideoCapture camera;
  cv::Mat image;
  cv_bridge::CvImagePtr frame;

  ros::Time last;
  ros::Duration period;
};


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "camera" );

  ros::NodeHandle nh( "~" );

  int camera_index = CameraDriver::DEFAULT_CAMERA_INDEX;
  nh.param( "camera_index", camera_index, camera_index );

  CameraDriver camera_driver( camera_index );

  while( ros::ok() )
  {
    camera_driver.capture();

    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}

