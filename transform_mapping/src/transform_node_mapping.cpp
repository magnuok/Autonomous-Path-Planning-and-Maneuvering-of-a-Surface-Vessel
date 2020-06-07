/**
 * Convert Twist msg from observer to odom msg for creating map. gmapping pkg requires odom msg.
 * Author: Magnus Knaedal
 * Date: 10.06.2020
 */

#include "transform_mapping/transform_node_mapping.hpp"



Transformer::Transformer(ros::NodeHandle nh){

twist_eta = geometry_msgs::Twist();
twist_nu = geometry_msgs::Twist();

sub_eta = nh.subscribe("observer/eta/ned", 1000, &Transformer::Callback_eta, this);
sub_ned = nh.subscribe("observer/nu/body", 1000, &Transformer::Callback_nu, this);
odom_pub = nh.advertise<nav_msgs::Odometry>( "odom", 1000); 

}


void Transformer::Callback_eta(const geometry_msgs::Twist& eta)
/**
 * Callback function for twist msg from observer. Publish odom msg.
 */

{
  float deg2rad = 3.14159265358979323846 / 180;

  nav_msgs::Odometry odom; 
  odom.header.frame_id = "odom";
  odom.header.stamp = ros::Time::now();
  tf2::Quaternion q;

  // Eta:
  q.setRPY(eta.angular.x * deg2rad, eta.angular.y * deg2rad, eta.angular.z * deg2rad);
  q.normalize();
  odom.pose.pose.position.x = eta.linear.x;
  odom.pose.pose.position.y = eta.linear.y;
  odom.pose.pose.position.z = eta.linear.z;
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = twist_nu.linear.x;
  odom.twist.twist.linear.y = twist_nu.linear.y;
  odom.twist.twist.linear.z = twist_nu.linear.z;
  odom.twist.twist.angular.x = twist_nu.angular.x;
  odom.twist.twist.angular.y = twist_nu.angular.y;
  odom.twist.twist.angular.z = twist_nu.angular.z;

  odom_pub.publish(odom);
}

void Transformer::Callback_nu(const geometry_msgs::Twist& msg)
{
  twist_nu = msg;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Transformer_mapping");
  ros::NodeHandle nh;
  Transformer trans(nh);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}