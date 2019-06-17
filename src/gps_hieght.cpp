#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/NavSatFix.h>

#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

double Kp = 0.2;
double Ki = 0;
double Kd = 0;
double kp,ki,kd;
double mass=1.2;
double g=9.8;
double hieght=0;
 
double des_hieght=588;
double throttle;

ros::Time last_time;
ros::Time current_time;

double Sample_time=1; //1 secs 

//////////////GPS Call Back---------------////////////////
void gps_calback(const sensor_msgs::NavSatFix& msg)
{
    hieght = msg.altitude;
    ROS_INFO_STREAM("CAllBACK :"<<hieght<<std::endl);
    
}
/////----------Error Structure------------------/////////////
struct error_data
{
    double error;
    double error_old;
    double error_sum;
    double error_diff;
    double last_input;
};

error_data e;

//////////////------------Int main-----------------//////////////////
int main(int argc, char **argv)
{
  // Create publishers, subscribers, etc.
  ros::init(argc, argv, "state_publisher");
    ros::NodeHandle nh;
    ros::Subscriber sonar_sub = nh.subscribe("/mavros/global_position/global", 1000, gps_calback);
    ros::Publisher rc_pub=nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);
    ros::Rate rate(10);
    //rc_override
    mavros_msgs::OverrideRCIn msg;
    //error data
    void set_tuning(double kpx,double kix, double kdx);
    int update_pid(double &set_point,double &input,ros::Time &time_now, ros::Time &time_last);
    void set_sample_time(double new_sample_time);
    e.error_sum=0;
    e.last_input=0;
    current_time=ros::Time::now();
    last_time=ros::Time::now();
    //set mode!!
    ros::ServiceClient cl = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "STABILIZE";
    
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
  // Do our own spin loop
 set_sample_time(0.01); //time in secs
  set_tuning(Kp,Ki,Kd);
  while (ros::ok())
  {
    // Do non-callback stuff
        int update=update_pid(des_hieght,hieght,current_time,last_time);
        
        //update throttle
        throttle= (mass*g)+update;
        if(throttle>MAXRC) throttle=MAXRC;
        else if(throttle<0) throttle=0;
        //publish throttle
        ROS_INFO_STREAM("Update: "<<update<<"Throttle: "<<throttle);
        msg.channels[0] = BASERC;     //Roll
        msg.channels[1] = BASERC;    //Pitch
        msg.channels[2] = throttle;   //Throttle
        msg.channels[3] = BASERC;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        rc_pub.publish(msg);
       
        rate.sleep();
    //usleep(100000);

    
  }
   ros::spinOnce();
}

int update_pid(double &set_point,double &input,ros::Time &time_now, ros::Time &time_last)
{
 int output;   
 //current_time=ros::Time::now();
 int time_change=(time_now-time_last).toSec();
 ROS_ERROR("Current Time: %d",time_change);    
 if(time_change>=Sample_time)
 { 
       time_last=time_now;
 }   
    e.error=set_point-input;
    e.error_sum+=(ki*e.error);
    e.error_diff=(input-e.last_input);
    ROS_INFO_STREAM("Error:"<<e.error<<"Sum:"<<e.error_sum<<" Diff:"<<e.error_diff);
  
    //calculate pid
    output= static_cast<int>((kp*e.error + e.error_sum - kd*e.error_diff)*mass);
    ROS_INFO_STREAM("Output: "<<output);
 

    e.last_input=input;
  
    return output;
}

void set_sample_time(double new_sample_time)
{
    if(new_sample_time>0)
    {
       /* double ratio=new_sample_time/Sample_time;
        ki*=ratio;
        kd/=ratio;*/
        Sample_time=new_sample_time;
    }

}
void set_tuning(double kpx,double kix, double kdx)
{
    kp = kpx;
    ki = kix;//*Sample_time;
    kd = kdx;///Sample_time;
    ROS_INFO_STREAM("PID Param Set-->"<<"kp="<<kp<<","<<"ki="<<ki<<","<<"kd="<<kd<<std::endl);
}