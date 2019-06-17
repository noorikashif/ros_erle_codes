#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>

static constexpr double _180_OVER_PI = 180.0 / 3.14159;

void subCallback(const geometry_msgs::PoseStamped& msg)
{
    tf::Quaternion q(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    double degree= yaw*_180_OVER_PI;
    ROS_INFO_STREAM("YAW"<<yaw<<std::endl<<"Degree"<<degree<<std::endl);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat_to_euler");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/aruco_single/pose", 1000, subCallback);
    ros::spin();    
    return 0;
}

