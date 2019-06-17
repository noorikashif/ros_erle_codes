#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Range.h>
#define Kp  25
#define Ki  5
#define Kd  1

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900
float hieght=0; 
int des_hieght=10;
double throttle;
void sonar_calback(const sensor_msgs::Range::ConstPtr& msg)
{
    hieght = msg->range;
}

struct error_data
{
    double error;
    double error_old;
    double error_sum;
    double error_diff;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pidOneD");
    ros::NodeHandle nh;
    ros::Subscriber sonar_sub = nh.subscribe("/sonar_down", 1000, sonar_calback);
    ros::Publisher rc_pub=nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);
    ros::Rate rate(10);
    //rc_override
    mavros_msgs::OverrideRCIn msg;
    //error data
    error_data e;
    e.error_sum=0;
    e.error_old=0;
    //set mode!!
    ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "LOITER";
    
    if(cl.call(srv_setMode))
    {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }
    else
    {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    //arm copter

    ros::ServiceClient arming_cl = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv))
    {
        ROS_INFO("ARM send ok %d", srv.response.success);
    }
    else
    {
        ROS_ERROR("Failed arming or disarming");
    }


    
    while(ros::ok())
    {
        //calculate error
        e.error=des_hieght-hieght;
        e.error_sum+=e.error;
        e.error_diff=e.error-e.error_old;
        //calculate pid
        int update= static_cast<int>(Kp*e.error + Ki*e.error_sum + Kd*e.error_diff);
        //update throttle
        throttle= BASERC + update;
        //publish throttle
        msg.channels[0] = BASERC;     //Roll
        msg.channels[1] = BASERC;    //Pitch
        msg.channels[2] = throttle;   //Throttle
        msg.channels[3] = BASERC;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        rc_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}