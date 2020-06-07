#ifndef CME_CME_ROS_H
#define CME_CME_ROS_H

/******** Includes ********/
#include "ros/ros.h"
#include "ros/time.h"
#include "nav_msgs/Odometry.h"
//#include "tf.h"
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Twist.h>


/******** Definitions ********/

#define LIN_ACTUATOR_UP 1
#define LIN_ACTUATOR_DOWN 2
#define LIN_ACTUATOR_NEUTRAL 0
#define LIN_ACTUATOR_HEADING_CONT 1

/******** Classes ********/
class Transformer
{
public:
    /* Constructor */
    explicit Transformer(ros::NodeHandle nh);

    /* Callback functions - used for dealing with incoming topics subscribed to by this node */
    void Callback_eta(const geometry_msgs::Twist& msg);
    void Callback_nu(const geometry_msgs::Twist& msg);

    void publish_odom();


    geometry_msgs::Twist twist_eta ;
    geometry_msgs::Twist twist_nu;

private:

    /* Node handler */
    ros::NodeHandle nh;

    /* Publishers */
    ros::Publisher odom_pub;

    /* Subscribers */
    ros::Subscriber sub_eta;
    ros::Subscriber sub_ned;

};


#endif