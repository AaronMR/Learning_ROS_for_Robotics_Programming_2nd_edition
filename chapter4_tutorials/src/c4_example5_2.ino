#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 range;
ros::Publisher range_pub("range", &range);

const int trigpin = 7;
const int echopin = 3;
long duration = 0;

void setup()
{
  nh.initNode();
  nh.advertise(range_pub);

}

void loop()
{
  range.data = ping();
  range_pub.publish(&range);
  nh.spinOnce();
  delay(100);
}

long ping()
{
// Send out PING))) signal pulse
  pinMode(trigpin, OUTPUT);
  pinMode(echopin,INPUT);
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  //Get duration it takes to receive echo
  duration = pulseIn(echopin, HIGH);
  //Convert duration into distance
  return duration /290/2;
}
