#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>


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


int throttle,roll,pitch;
double change_roll,old_roll=0;
struct error_data
{
    double e;
    double e_old;
    double e_sum;
    double e_diff;
};
struct data
{
    double input;
    double SetPoint;
    int mode;
};
struct pid
{
    double kp;
    double ki;
    double kd;
};
///-----------------Strut variables----------/////////    
data alt;
data rollX;
data pitchY;
//error_data error;
error_data alt_error;
error_data rollX_error;
error_data pitchY_error;
pid alt_gain, rollx_gain,pitchY_gain;

/////////////////----------------------PID GAINS-------------------------------/////////////
double Kp=100; //100
double Ki=1.5; //1.5
double Kd=10; //10
//////////---------------Call BAck ---------------------///////////////
void gps_callback(const sensor_msgs::NavSatFix& msg)
{
   alt.input=msg.altitude;
    ROS_INFO_STREAM("Hieght="<<alt.input<<std::endl);
}

void aruco_callback(const geometry_msgs::Vector3Stamped& pos)
{  // alt.input=pos.vector.z;
    rollX.input=pos.vector.x;
    pitchY.input=pos.vector.y;
}
/////////////-------------------------------------////////////////////////
int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "stabilize_node",ros::init_options::NoSigintHandler);
    signal(SIGINT,mySigIntHandler);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown",shutdownCallback);
    //////////////-------FUNCTION DEFINATION--------////////////////////

    int update_pid(data mode_input, error_data mode_error, pid mode_gain);

    mavros_msgs::OverrideRCIn msg;
    //-------------------INitialize struct---------------//////////
        alt.SetPoint=588; //588// 1.4
        alt.input=584; //584
        alt.mode=1;
        rollX.SetPoint=0;
        rollX.mode=-1;
        pitchY.SetPoint=0;
        pitchY.mode=-1;
        //error_data error;
        alt_error.e_sum=0;
        alt_error.e_old=0;
        rollX_error.e_sum=0;
        rollX_error.e_old=0;
        pitchY_error.e_sum=0;
        pitchY_error.e_old=0;
        // PID GAINS//
        alt_gain.kp=100;
        alt_gain.ki=1.5;
        alt_gain.kd=10;

        rollx_gain.kp=50;
        rollx_gain.ki=0;//3
        rollx_gain.kd=5;
        
        pitchY_gain.kp=50;//130
        pitchY_gain.ki=0;//3
        pitchY_gain.kd=5;//15
            


    ///////////-------------------------////////////
    ros::NodeHandle nh;
    ros::Subscriber gps = nh.subscribe("/mavros/global_position/global", 1000, gps_callback);
    ros::Subscriber aruco = nh.subscribe("/aruco_single/position", 1000, aruco_callback);
    
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

    throttle=1500+update_pid(alt,alt_error,alt_gain);
    roll=1500+update_pid(rollX,rollX_error,rollx_gain);
    pitch=1500+update_pid(pitchY,pitchY_error,pitchY_gain);
    if(throttle>MAXRC)throttle=MAXRC;
    else if(throttle<MINRC)throttle=MINRC;
    if ((rollX.input<0.09)&&(rollX.input>(-0.09)))
    roll=BASERC;
    else if(roll>MAXRC)roll=MAXRC;
    else if(roll<MINRC)roll=MINRC;    
    if ((pitchY.input<0.09)&&(pitchY.input>(-0.09)))
    pitch=BASERC;
    if(pitch>MAXRC)pitch=MAXRC;
    else if(pitch<MINRC)pitch=MINRC;
    ROS_INFO_STREAM("Throttle: "<<throttle);
    ROS_INFO_STREAM("Roll: "<<roll);
    ROS_INFO_STREAM("Pitch: "<<pitch);
    msg.channels[0] = roll;     //Roll
    msg.channels[1] = pitch;    //Pitch
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
    srv_land.request.altitude = alt.input;
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



int update_pid(data mode_input, error_data mode_error, pid mode_gain)
{
    int output;
    mode_error.e=(mode_input.mode)*(mode_input.SetPoint-mode_input.input);

    mode_error.e_diff=(mode_error.e-mode_error.e_old);
    //-----CLamping I-Term----//
    if(mode_error.e_sum<MAXRC)
    mode_error.e_sum+=(mode_gain.ki*mode_error.e);
    if(mode_error.e_sum<(-2000))
    mode_error.e_sum=-2000;
    ////-------------------/////
    double pid=mode_gain.kp*(mode_error.e)+(mode_error.e_sum)-(mode_gain.kd*mode_error.e_diff);
    output=static_cast<int>(pid);
    ROS_INFO_STREAM("output"<<mode_error.e<<std::endl);
    mode_error.e_old=mode_error.e;
    return output;
}