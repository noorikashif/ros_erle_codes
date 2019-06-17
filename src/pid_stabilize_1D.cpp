#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/NavSatFix.h>



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

//////////////-------------GLObal Variables-----------------//////////////////
#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

double hieght=584;
double des_hieght=587;
int throttle;

struct error_data
{
    double e;
    double e_old;
    double e_sum;
    double e_diff;
};

error_data error;

/////////////////----------------------PID GAINS-------------------------------/////////////
double Kp=100; //100
double Ki=1.5; //1.5
double Kd=10; //10
//////////------------------------------------///////////////
void sonar_callback(const sensor_msgs::NavSatFix& msg)
{
    hieght=msg.altitude;
    ROS_INFO_STREAM("Hieght="<<hieght<<std::endl);
}

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "stabilize_node",ros::init_options::NoSigintHandler);
    signal(SIGINT,mySigIntHandler);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown",shutdownCallback);
    //////////////-------FUNCTION DEFINATION--------////////////////////

    int update_pid(double input);

    mavros_msgs::OverrideRCIn msg;
    error.e_sum=0;
    error.e_old=0;
    ///////////-------------------------////////////
    ros::NodeHandle nh;
    ros::Subscriber sonar = nh.subscribe("/mavros/global_position/global", 1000, sonar_callback);
    ros::Publisher rc_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    ros::Rate rate(10);
///---------------------Set MOde and ARM----------------------//////

    ros::ServiceClient cl=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setmode;
    srv_setmode.request.base_mode=0;
    srv_setmode.request.custom_mode="LOITER";
    if(cl.call(srv_setmode))
    {
         ROS_INFO("setmode send ok %d value:", srv_setmode.response.success);
    }
    else
    {
        ROS_ERROR("Failed Setmode");
        return -1;
    }
    //arming
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

///////////////------------------------////////////////

    while(!g_request_shutdown)
    {

    throttle=1500+update_pid(hieght);
    
    if(throttle>MAXRC)throttle=MAXRC;
    else if(throttle<MINRC)throttle=MINRC;
        
    
    ROS_INFO_STREAM("Throttle: "<<throttle);
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
    
    ros::spinOnce();
        
    }
// Pre Shutdown Task::: LAND/////////

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
return 0;
}



int update_pid(double input)
{
    int output;
    error.e=des_hieght- input;

    error.e_diff=(error.e-error.e_old);
    //-----CLamping I-Term----//
    if(error.e_sum<MAXRC)
    error.e_sum+=(Ki*error.e);
    if(error.e_sum<(-2000))
    error.e_sum=-2000;
    ////-------------------/////
    double pid=Kp*(error.e)+(error.e_sum)-(Kd*error.e_diff);
    output=static_cast<int>(pid);
    ROS_INFO_STREAM("output"<<error.e<<std::endl);
    error.e_old=error.e;
    return output;
}