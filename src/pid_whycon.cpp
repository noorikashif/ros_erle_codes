#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Range.h>

geometry_msgs::Pose test;
geometry_msgs::PoseArray test1;
double kp_t=300;   
double ki_t=0.6; 
double kd_t=1; 
double kp_r=300;   
double ki_r=0.6; 
double kd_r=1; 
double kp_p=300;   
double ki_p=0.6; 
double kd_p=1;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}
/////////////////////////////////////////////////////////////////////

double error,error_sum,error_diff;
#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900
float orig_x=0,orig_y=0,orig_z=0; 
int des_x=1,des_y=1,des_z=3;
double throttle;
double roll;
double pitch;
double t_old=0,r_old=0,p_old=0;
void why_calback(const geometry_msgs::PoseArray& msg)
{
    
    orig_x = msg.poses[0].position.x;
    orig_y = msg.poses[0].position.y;
    orig_z = msg.poses[0].position.z;
    ROS_INFO_STREAM("Org_x="<<orig_x);
}


double pid(double kp, double ki, double kd, double des, double orig, double max, double min, double base, double ret, double old)
{
        error=des-orig;
        if(ret<max && ret>min)
        error_sum+=error;
        error_diff=error-old;
        //calculate pid
        //if(e.error<0 && e.error_sum>0)
        //e.error_sum=0;
        int update=kp*error + ki*error_sum + kd*error_diff;
        //update throttle
        ret= base + update;
        if(ret>max)
        ret=max;
        if(ret<min)
        ret=min;
        return(ret);
}
int main(int argc, char** argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "MyNode", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);
//////////////////////////////////////usr
////////////////////////////////////////////usr
////////////////////////////////////usr

  // Create publishers, subscribers, etc.
    ros::NodeHandle nh;
    ros::Subscriber why_sub = nh.subscribe("/whycon/poses", 1000, why_calback);
    ros::Publisher rc_pub=nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",10);
    ros::Rate rate(50);
    //rc_override
    mavros_msgs::OverrideRCIn msg;
    //error data
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
  // Do our own spin loop
  while (!g_request_shutdown)
  {
    //height
//double kp, double ki, double kd, double des, double orig, double max, double min, double base, double ret, double old)

       throttle=pid(kp_t,ki_t,kd_t,orig_z,des_z,2000,1000,1500,throttle, t_old);
       t_old=error;
       roll=pid(kp_r,ki_r,kd_r,orig_x,des_x,2000,1000,1500,roll, r_old);
       r_old=error;
       pitch=pid(kp_p,ki_p,kd_p,orig_y,des_y,2000,1000,1500,roll, p_old);
       p_old=error;



        msg.channels[0] = roll;     //Roll
        msg.channels[1] = pitch;    //Pitch
        msg.channels[2] = throttle;   //Throttle
        msg.channels[3] = 1500;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;
       
        rc_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    //usleep(100000);
         
    //rate.sleep();
  }

  // Do pre-shutdown tasks
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = orig_z;
    srv_land.request.latitude =  0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land))
    {
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }
    else
    {
        ROS_ERROR("Failed Land");
    }

  ros::shutdown();
}
