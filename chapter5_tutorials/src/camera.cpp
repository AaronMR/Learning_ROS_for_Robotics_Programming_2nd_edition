
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <chapter5_tutorials/CameraConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>
#include <string>


class CameraDriver
{
public:

  typedef chapter5_tutorials::CameraConfig Config;
  typedef camera_info_manager::CameraInfoManager CameraInfoManager;

  enum Default
  {
    DEFAULT_CAMERA_INDEX = 0
  };

  static const double DEFAULT_FPS;
  static const char* DEFAULT_CAMERA_NAME;

  CameraDriver()
  :
    nh( "~" ),
    it( nh ),
    camera_pub( it.advertiseCamera( "image_raw", 1 ) ),
    camera_info_manager( nh ),
    server( nh ),
    reconfiguring( false )
  {
    nh.param<int>( "camera_index", camera_index, DEFAULT_CAMERA_INDEX );
    nh.param<std::string>( "camera_name", camera_name, DEFAULT_CAMERA_NAME );
    nh.param<double>( "fps", fps, DEFAULT_FPS );

    camera.release();   // release because automatic constructor may have open it!
    if( not camera.open( camera_index ) )
    {
      ROS_ERROR_STREAM( "Failed to open camera device!" );
      ros::shutdown();
    }

    setCameraName( camera_info_manager, camera_name );

    frame = boost::make_shared< cv_bridge::CvImage >();
    frame->encoding = sensor_msgs::image_encodings::BGR8;

    camera_info = boost::make_shared< sensor_msgs::CameraInfo >();

    server.setCallback( boost::bind( &CameraDriver::reconfig, this, _1, _2 ) );

    timer = nh.createTimer( ros::Duration( 1. / fps ), &CameraDriver::capture, this );
  }

  ~CameraDriver()
  {
    camera.release();
  }

  void reconfig( Config& newconfig, uint32_t level )
  {
    reconfiguring = true;
    boost::mutex::scoped_lock lock( mutex );

    if( camera_index != newconfig.camera_index )
    {
      camera.release();
      if( camera.open( newconfig.camera_index ) )
      {
        camera_index = newconfig.camera_index;
        ROS_INFO_STREAM( "New camera (index = " << camera_index << ") opened successfully!" );
      }
      else
      {
        ROS_WARN_STREAM( "Failed to open new camera (index = " << newconfig.camera_index << ")!" );

        if( not camera.open( camera_index ) )
        {
          ROS_ERROR_STREAM( "Failed to re-open previous camera (index = " << camera_index << ")!" );
          ros::shutdown();
        }
      }
    }

    setCameraInfo( camera_info_manager , config.camera_info_url, newconfig.camera_info_url );

    newconfig.frame_width  = setProperty( camera, CV_CAP_PROP_FRAME_WIDTH , newconfig.frame_width  );
    newconfig.frame_height = setProperty( camera, CV_CAP_PROP_FRAME_HEIGHT, newconfig.frame_height );
    //newconfig.fps          = setProperty( camera, CV_CAP_PROP_FPS         , newconfig.fps          );
    newconfig.brightness   = setProperty( camera, CV_CAP_PROP_BRIGHTNESS  , newconfig.brightness   );
    newconfig.contrast     = setProperty( camera, CV_CAP_PROP_CONTRAST    , newconfig.contrast     );
    newconfig.saturation   = setProperty( camera, CV_CAP_PROP_SATURATION  , newconfig.saturation   );
    newconfig.hue          = setProperty( camera, CV_CAP_PROP_HUE         , newconfig.hue          );
    newconfig.gain         = setProperty( camera, CV_CAP_PROP_GAIN        , newconfig.gain         );
    newconfig.exposure     = setProperty( camera, CV_CAP_PROP_EXPOSURE    , newconfig.exposure     );

    //setFOURCC( camera, newconfig.fourcc );

    frame->header.frame_id = newconfig.frame_id;

    if( fps != newconfig.fps )
    {
      fps = newconfig.fps;
      timer.setPeriod( ros::Duration( 1. / fps ) );
      ROS_INFO_STREAM( "Sampling timer period set to " << fps << "FPS" );
    }

    config = newconfig;
    reconfiguring = false;
  }

  void capture( const ros::TimerEvent& te )
  {
    if( not reconfiguring )
    {
      boost::mutex::scoped_lock lock( mutex );

      camera >> frame->image;
      if( not frame->image.empty() )
      {
        frame->header.stamp = ros::Time::now();

        *camera_info = camera_info_manager.getCameraInfo();
        camera_info->header = frame->header;

        camera_pub.publish( frame->toImageMsg(), camera_info );
      }
    }
  }

private:

  void setCameraName( CameraInfoManager& camera_info_manager, const std::string& camera_name )
  {
    if( not camera_info_manager.setCameraName( camera_name ) )
    {
      ROS_ERROR_STREAM( "Invalid camera name '" << camera_name << "'" );
      ros::shutdown();
    }
  }

  void setCameraInfo( CameraInfoManager& camera_info_manager, const std::string& camera_info_url, std::string& camera_info_url_new )
  {
    if( camera_info_url != camera_info_url_new )
    {
      if( camera_info_manager.validateURL( camera_info_url_new ) )
      {
        camera_info_manager.loadCameraInfo( camera_info_url_new );
      }
      else
      {
        camera_info_url_new = camera_info_url;
      }
    }
  }

  double setProperty( cv::VideoCapture& camera, int property, double value )
  {
    if( camera.set( property, value ) )
    {
      double current_value = camera.get( property );
      ROS_WARN_STREAM(
        "Failed to set property #" << property << " to " << value <<
        " (current value = " << current_value << ")"
      );
      return current_value;
    }

    return value;
  }

  std::string setFOURCC( cv::VideoCapture& camera, std::string& value )
  {
    ROS_ASSERT_MSG( value.size() == 4, "Invalid FOURCC codec" );

    int property = CV_CAP_PROP_FOURCC;
    int fourcc = CV_FOURCC( value[0], value[1], value[2], value[3] );
    if( camera.set( property, fourcc ) )
    {
      fourcc = camera.get( property );
      std::string current_value = fourccToString( fourcc );
      ROS_WARN_STREAM(
        "Failed to set FOURCC codec to '" << value <<
        "' (current value = '" << current_value << "' = " << fourcc << ")"
      );
      return current_value;
    }

    return value;
  }

  std::string fourccToString( int fourcc )
  {
    std::string str( 4, ' ' );

    for( size_t i = 0; i < 4; ++i )
    {
      str[i] = fourcc & 255;
      fourcc >>= 8;
    }

    return str;
  }

private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::CameraPublisher camera_pub;
  sensor_msgs::CameraInfoPtr camera_info;
  CameraInfoManager camera_info_manager;
  std::string camera_name;

  Config config;
  dynamic_reconfigure::Server< Config > server;
  bool reconfiguring;
  boost::mutex mutex;

  cv::VideoCapture camera;
  cv_bridge::CvImagePtr frame;

  ros::Timer timer;

  int camera_index;
  double fps;
};

const double CameraDriver::DEFAULT_FPS = 30.;
const char* CameraDriver::DEFAULT_CAMERA_NAME = "webcam";


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "camera" );

  CameraDriver camera_driver;

  while( ros::ok() )
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}

