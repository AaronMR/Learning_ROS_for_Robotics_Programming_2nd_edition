#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <chapter2_tutorials/chapter2_Config.h>

void callback(chapter2_tutorials::chapter2_Config &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example4");

  dynamic_reconfigure::Server<chapter2_tutorials::chapter2_Config> server;
  dynamic_reconfigure::Server<chapter2_tutorials::chapter2_Config>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
