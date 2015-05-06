
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <chapter3_tutorials/DynamicParamConfig.h>

class DynamicParamServer
{
public:
  DynamicParamServer()
  {
    _cfg_server.setCallback(boost::bind(&DynamicParamServer::callback, this, _1, _2));
  }

  void callback(chapter3_tutorials::DynamicParamConfig& config, uint32_t level)
  {
    ROS_INFO_STREAM(
        "New configuration received with level = " << level << ":\n" <<
        "bool   = " << config.bool_param << "\n" <<
        "int    = " << config.int_param << "\n" <<
        "double = " << config.double_param << "\n" <<
        "string = " << config.string_param
        );
  }

private:
  dynamic_reconfigure::Server<chapter3_tutorials::DynamicParamConfig> _cfg_server;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example6");

  DynamicParamServer dps;

  while(ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}
