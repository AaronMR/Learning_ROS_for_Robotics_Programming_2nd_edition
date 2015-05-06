#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<stdio.h>

using namespace std;

class Dynamixel{
  private:
    ros::NodeHandle n;
    ros::Publisher pub_n;
  public:
    Dynamixel();
    int moveMotor(double position);
};

Dynamixel::Dynamixel(){
  pub_n = n.advertise<std_msgs::Float64>("/tilt_controller/command",1);
}
int Dynamixel::moveMotor(double position)
{
  std_msgs::Float64 aux;
  aux.data = position;
  pub_n.publish(aux);
  return 1;
}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "example4_move_motor");
  Dynamixel motors;

  float counter = -180;
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    if(counter < 180)
    {
      motors.moveMotor(counter*3.14/180);
      counter++;
    }else{
      counter = -180;
    }
    loop_rate.sleep();
  }
}
