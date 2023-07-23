#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

geometry_msgs::Twist twist;

bool update = false;
void joyStateCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->axes[1] != twist.linear.x) {
    twist.linear.x = msg->axes[1];
    update = true;
  }
  if (msg->axes[0] != twist.angular.z) {
    twist.angular.z = msg->axes[0];
    update = true;
  }
  //ROS_INFO("joyStateCallback() called %f %f", twist.linear.x,twist.angular.z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wheebot_joy");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub =
      nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber joy_sub =
      nh.subscribe("/joy", 1000, joyStateCallback);

  ros::Rate loop_rate(50);
  while (ros::ok()) {
    if (update) {
      // ROS_INFO("publish twist");
      cmd_pub.publish(twist);
      update = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}