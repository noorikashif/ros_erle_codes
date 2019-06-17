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
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
int flag=0,count=0;
geometry_msgs::Pose test;
double kp_t=400;   
double ki_t=0.2; 
double kd_t=20; 
double kp_r=400;   
double ki_r=0.2; 
double kd_r=20; 
double kp_p=400;   
double ki_p=0.2; 
double kd_p=20;
double kp_theta=1000;   
double ki_theta=0.01; 
double kd_theta=10;
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
//#define MINRC   1100
//#define BASERC  1500
//#define MAXRC   1900
float orig_x=0,orig_y=0,orig_z=0,orig_theta=0; 
float des_x=0.6,des_y=0.6,des_z=4,des_theta=-1.5;
double throttle;
double roll;
double pitch;
double yaw;
double t_old=0,r_old=0,p_old=0,theta_old=0,t_sum=0,r_sum=0,p_sum=0,theta_sum=0;

void imuCallback_(const sensor_msgs::Imu::ConstPtr msg) 
{
tf::Quaternion q(
msg->orientation.x,
msg->orientation.y,
msg->orientation.z,
msg->orientation.w);
tf::Matrix3x3 m(q);
double roll_imu, pitch_imu, yaw_imu;
m.getRPY(roll_imu, pitch_imu, yaw_imu);
orig_theta=yaw_imu;
//pose2d.theta = yaw;
//pub_pose_.publish(pose2d);
}

void why_calback(const geometry_msgs::PoseArray& msg)
{
    //test=msg->poses;
    orig_x = msg.poses[0].position.x;
    orig_y = msg.poses[0].position.y;
    orig_z = msg.poses[0].position.z;
}


double pid(double kp, double ki, double kd, double des, double orig, double max, double min, double base, double ret, double old, double sum)
{
        error=des-orig;
        error_sum=sum;
        if(ret<max && ret>min)
        error_sum+=error;
        error_diff=error-old;
        int update=kp*error + ki*error_sum + kd*error_diff;
        //ROS_INFO_STREAM("error:"<<error<<" prop:"<<kp*error<<" update:"<<update<<" integ"<<ki*error_sum<<" diff:"<< kd*error_diff<<std::endl);
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
    ros::Publisher rc_pub=nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",20);
    ros::Subscriber sub_odom_ = nh.subscribe("/mavros/imu/data", 1, imuCallback_);
    ros::Rate rate(150);
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

       throttle=pid(kp_t,ki_t,kd_t,orig_z,des_z,2000,1000,1500,throttle, t_old, t_sum);
       t_old=error;
       t_sum=error_sum;


       if(orig_z>=6 || orig_theta>-1.25 || orig_theta<-1.75)
       roll=1500;
       else
       {
        roll=pid(kp_r,ki_r,kd_r,orig_x,des_x,1800,1200,1500,roll, r_old, r_sum);
        r_old=error;
        r_sum=error_sum;
       }


       if(orig_z>=6 || orig_theta>-1.25 || orig_theta<-1.75)
       pitch=1500;
       else
       {
        pitch=pid(kp_p,ki_p,kd_p,orig_y,des_y,1800,1200,1500, pitch, p_old, p_sum);
        p_old=error;
        p_sum=error_sum;
       }

      
       if(orig_z>=6)
       yaw=1500;
       else
        {
          yaw=pid(kp_theta,ki_theta,kd_theta,orig_theta,des_theta,1800,1200,1500, yaw, theta_old , theta_sum);
          theta_old=error;
          theta_sum=error_sum;
          if(orig_theta>-1.51 && orig_theta <-1.49 && flag==1)
           {                      //ROS_INFO("ok"<<theta_sum);
           ROS_INFO_STREAM("error:"<<theta_sum<<std::endl);
              count--;
              if(theta_sum>50)
              yaw=1450;
              if(theta_sum<-50)
              yaw=1650;
              if(count==0)
              {
              flag=0;
              theta_sum=0;
              }                                   
            }
          if(orig_theta<-1.51 || orig_theta >-1.49)
          {
          flag=1;
          count=500;
          }
        }

       
         

        msg.channels[0] = roll;       //Roll
        msg.channels[1] = pitch;       //Pitch
        msg.channels[2] = throttle;   //Throttle
        msg.channels[3] = yaw;       //Yaw
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
    msg.channels[0] = 1500;       //Roll
    msg.channels[1] = 1500;
    msg.channels[3] = 1500; 
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