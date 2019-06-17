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


#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

double Kp = 5;
double Ki = 0;
double Kd = 0;
double kp,ki,kd;

double hieght=0;
 
double des_hieght=10;
double throttle;

ros::Time last_time;
ros::Time current_time;

double Sample_time=1; //1 secs 

//////////////Sonar Call Back---------------////////////////
void sonar_calback(const sensor_msgs::Range::ConstPtr& msg)
{
    hieght = msg->range;
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
    void set_tuning(double kpx,double kix, double kdx);
    int update_pid(double set_point,double input);
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
 //set_sample_time(0.01); time in secs
  set_tuning(Kp,Ki,Kd);
  while (!g_request_shutdown)
  {
    // Do non-callback stuff
        int update=update_pid(des_hieght,hieght);
        
        //update throttle
        throttle=  update;
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

int update_pid(double set_point,double input)
{
 int output;   
 current_time=ros::Time::now();
 int time_change=(current_time-last_time).toSec();
 
 //if(time_change>=Sample_time)
 //{    
    e.error=set_point-input;
    e.error_sum+=(ki*e.error);
    e.error_diff=input-e.last_input;
    ROS_INFO_STREAM("Error:"<<e.error<<"Sum:"<<e.error_sum<<" Diff:"<<e.error_diff);
  
    //calculate pid
    output= static_cast<int>(kp*e.error + e.error_sum - kd*e.error_diff);
    ROS_INFO_STREAM("Output: "<<output);
 

    e.last_input=input;
    last_time=current_time;
  //}
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
    ki = kix*Sample_time;
    kd = kdx/Sample_time;
    ROS_INFO_STREAM("PID Param Set-->"<<"kp="<<kp<<","<<"ki="<<ki<<","<<"kd="<<kd<<std::endl);
}