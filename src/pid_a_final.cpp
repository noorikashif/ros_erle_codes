#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Range.h>

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

#define Kp  100 //70 //68  
#define Ki  1.5 //1 //0.6
#define Kd  10 //20  //10

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
  // Do our own spin loop
  while (!g_request_shutdown)
  {
    // Do non-callback stuff
        e.error=des_hieght-hieght;
        if(throttle<2000)
        e.error_sum+=e.error;
        e.error_diff=e.error-e.error_old;
        //calculate pid
        //if(e.error<0 && e.error_sum>0)
        //e.error_sum=0;
        int update= static_cast<int>(Kp*e.error + Ki*e.error_sum + Kd*e.error_diff);
        //update throttle
        throttle= BASERC + update;
        if(throttle>2000)
        throttle=2000;
     //       ROS_INFO_STREAM(" Error:"<<kp*e.error<<" Sum:"<<ki*e.error_sum<<" Diff:"<<-kd*e.error_diff<<" height:"<<hieght);
        //publish throttle
        msg.channels[0] = 1500;     //Roll
        msg.channels[1] = 1500;    //Pitch
        msg.channels[2] = throttle;   //Throttle
        msg.channels[3] = 1500;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;
        e.error_old=e.error;
        rc_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    //usleep(100000);
         
    rate.sleep();
  }

  // Do pre-shutdown tasks
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = hieght;
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