
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <chapter5_tutorials/CameraStereoConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>
#include <string>


class CameraDriver
{
public:

	typedef chapter5_tutorials::CameraStereoConfig Config;
    typedef camera_info_manager::CameraInfoManager CameraInfoManager;

    enum
    {
        LEFT        = 0,
        RIGHT       = 1,
        NUM_CAMERAS
    };

    static const int DEFAULT_CAMERA_INDEX[NUM_CAMERAS];
    static const double DEFAULT_FPS;
    static const char* DEFAULT_CAMERA_NAME;

    static const std::string CameraString[NUM_CAMERAS];

	CameraDriver()
	:
		nh( "~" ),
        camera_nh( "stereo" ),
        it( nh ),
        server( nh ),
		reconfiguring( false )
	{
        for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            camera[i].release();
        }

        for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            // We assume the indexes are different for all cameras.
            nh.param<int>( "camera_index_" + CameraString[i], camera_index[i], DEFAULT_CAMERA_INDEX[i] );

            if( camera[i].open( camera_index[i] ) )
            {
                ROS_INFO_STREAM( CameraString[i] << " camera (index = " << camera_index[i] << ") successfully opened!" );
            }
            else
            {
                ROS_ERROR_STREAM( "Failed to open " << CameraString[i] << " camera (index = " << camera_index[i] << ")!" );
                ros::shutdown();
            }
        }

		nh.param<std::string>( "camera_name", camera_name, DEFAULT_CAMERA_NAME );
		nh.param<double>( "fps", fps, DEFAULT_FPS );
        
		for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            single_camera_nh[i] = ros::NodeHandle( camera_nh, CameraString[i] );

            camera_info_manager[i] = boost::make_shared< CameraInfoManager >( single_camera_nh[i] );
            setCameraName( *camera_info_manager[i], camera_name + "_" + CameraString[i] );

		    frame[i] = boost::make_shared< cv_bridge::CvImage >();
		    frame[i]->encoding  = sensor_msgs::image_encodings::BGR8;

            camera_pub[i] = it.advertiseCamera( CameraString[i] + "/image_raw", 1 );
        }

		camera_info = boost::make_shared< sensor_msgs::CameraInfo >();

		server.setCallback( boost::bind( &CameraDriver::reconfig, this, _1, _2 ) );

		timer = nh.createTimer( ros::Duration( 1. / fps ), &CameraDriver::capture, this );
	}

	~CameraDriver()
	{
        for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            camera[i].release();
        }
	}

	void reconfig( Config& newconfig, uint32_t level )
	{
		reconfiguring = true;
		boost::mutex::scoped_lock lock( mutex );

        // Resolve frame ID using tf_prefix parameter:
        if( newconfig.frame_id == "" )
        {
            newconfig.frame_id = "camera";
        }
        std::string tf_prefix = tf::getPrefixParam( nh );
        ROS_DEBUG_STREAM( "tf_prefix = " << tf_prefix );
        newconfig.frame_id = tf::resolve( tf_prefix, newconfig.frame_id );

        setCameraInfo( *camera_info_manager[LEFT] , config.camera_info_url_left , newconfig.camera_info_url_left  );
        setCameraInfo( *camera_info_manager[RIGHT], config.camera_info_url_right, newconfig.camera_info_url_right );

        for( size_t i = 0; i < NUM_CAMERAS; ++i )
        {
            newconfig.frame_width  = setProperty( camera[i], CV_CAP_PROP_FRAME_WIDTH , newconfig.frame_width  );
            newconfig.frame_height = setProperty( camera[i], CV_CAP_PROP_FRAME_HEIGHT, newconfig.frame_height );
            //newconfig.fps          = setProperty( camera[i], CV_CAP_PROP_FPS         , newconfig.fps          );
            newconfig.brightness   = setProperty( camera[i], CV_CAP_PROP_BRIGHTNESS  , newconfig.brightness   );
            newconfig.contrast     = setProperty( camera[i], CV_CAP_PROP_CONTRAST    , newconfig.contrast     );
            newconfig.saturation   = setProperty( camera[i], CV_CAP_PROP_SATURATION  , newconfig.saturation   );
            newconfig.hue          = setProperty( camera[i], CV_CAP_PROP_HUE         , newconfig.hue          );
            newconfig.gain         = setProperty( camera[i], CV_CAP_PROP_GAIN        , newconfig.gain         );
            newconfig.exposure     = setProperty( camera[i], CV_CAP_PROP_EXPOSURE    , newconfig.exposure     );

            //setFOURCC( camera[i], newconfig.fourcc );

            frame[i]->header.frame_id = newconfig.frame_id;
        }

        if( fps != newconfig.fps )
        {
            fps = newconfig.fps;
            timer.setPeriod( ros::Duration( 1. / fps ) );
        }

		config = newconfig;
		reconfiguring = false;
	}

	void capture( const ros::TimerEvent& te )
	{
		if( not reconfiguring )
		{
			boost::mutex::scoped_lock lock( mutex );

            for( size_t i = 0; i < NUM_CAMERAS; ++i )
            {
			    camera[i] >> frame[i]->image;
                if( frame[i]->image.empty() ) return;
            }

            ros::Time now = ros::Time::now();

            for( size_t i = 0; i < NUM_CAMERAS; ++i )
            {
				frame[i]->header.stamp = now;

			    *camera_info = camera_info_manager[i]->getCameraInfo();
				camera_info->header = frame[i]->header;

				camera_pub[i].publish( frame[i]->toImageMsg(), camera_info );
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
                "' (current value = '" << current_value << "' = " <<  fourcc << ")"
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
	ros::NodeHandle nh, camera_nh, single_camera_nh[NUM_CAMERAS];
	image_transport::ImageTransport it;
	image_transport::CameraPublisher camera_pub[NUM_CAMERAS];
	sensor_msgs::CameraInfoPtr camera_info;
    boost::shared_ptr< CameraInfoManager > camera_info_manager[NUM_CAMERAS];
	std::string camera_name;

	Config config;
	dynamic_reconfigure::Server< Config > server;
	bool reconfiguring;
	boost::mutex mutex;

    cv::VideoCapture camera[NUM_CAMERAS];
	cv_bridge::CvImagePtr frame[NUM_CAMERAS];

	ros::Timer timer;

	int camera_index[NUM_CAMERAS];
	double fps;
};

const int CameraDriver::DEFAULT_CAMERA_INDEX[NUM_CAMERAS] = {0, 1};
const double CameraDriver::DEFAULT_FPS = 30.;
const char* CameraDriver::DEFAULT_CAMERA_NAME = "logitech_c120";

const std::string CameraDriver::CameraString[NUM_CAMERAS] = {"left", "right"};


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "camera_stereo" );

	CameraDriver camera_driver;

	while( ros::ok() )
	{
		ros::spin();
	}

	return EXIT_SUCCESS;
}

