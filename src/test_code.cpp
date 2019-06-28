#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#define MAX_VEL 2
#define BASE_VEL 0
#define MIN_VEL -2
static constexpr double _180_OVER_PI = 180.0 / 3.14159;

class PID
{
  public:

    PID(double kp, double ki, double kd);
    ~PID()
    {
      _error_sum=0;
      _old_error=0;
    };
    double _error;
    double update_PID(double set_point, double input,int mode);
   
  private:
    double _Kp;
    double _Ki;
    double _Kd;
    
    double _error_sum;
    double _error_diff;
    double _old_error;
    int _Iflag;


};
PID::PID(double kp, double ki, double kd)
{
  _Kp=kp;
  _Ki=ki;
  _Kd=kd;
  _old_error=0;
  _error_sum=0;
  _error_diff=0;
}
double PID::update_PID(double set_point, double input,int mode)
{
  double output;
  _error=(mode)*(set_point-input);
  _error_diff=_error-_old_error;
  if(_error_sum<MAX_VEL)
  {_error_sum+=_Ki*_error;
    _Iflag=1;
  }
  if(_error_sum<MIN_VEL)
  {_error_sum=MIN_VEL;
    _Iflag=0;
  }
  output=(_Kp*_error+(_Iflag*_error_sum)-(_Kd*_error_diff));
  return output;
}
struct data
{
    double input;
    double SetPoint;
    int mode;
    void init_struct(double sp, double in, int md)
    {
      input=in;
      SetPoint=sp;
      mode=md;
    };
};
data alt;
data VelX;
data VelY;
double linear_vel,theta=0;
ros::Time aruco_time;
void odom_callback(const nav_msgs::Odometry& msg)
{
  linear_vel=msg.twist.twist.linear.x;
  if(linear_vel<0.02)
  {
    linear_vel=0;
  }
}
void gps_callback(const sensor_msgs::NavSatFix& msg)
{   
       alt.input=msg.altitude;
    ROS_INFO_STREAM("Hieght="<<alt.input<<std::endl);
}

void aruco_callback(const geometry_msgs::PoseStamped& pos)
{   tf::Quaternion q(pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z,pos.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    theta= yaw;
    aruco_time=pos.header.stamp;
    VelX.input=pos.pose.position.x;
    VelY.input=pos.pose.position.y;
    
}


double throttle, roll, pitch;
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



int main(int argc, char** argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "MyNode", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  // Create publishers, subscribers, etc.
  ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odom_callback);
    ros::Subscriber sub = nh.subscribe("/aruco_single/pose", 1000, aruco_callback);
    ros::Subscriber gps = nh.subscribe("/mavros/global_position/global", 1000, gps_callback);
  // ros::Subscriber aruco = nh.subscribe("/aruco_single/position", 1000, aruco_callback);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Rate rate(10);
///////////////////////-------------------------VARIABLES----------------------//////////////

PID alt_z(0.5,0,0.03); // kp, ki, kd
PID vel_x(0.3,0,0.02);
PID vel_y(0.3,0,0.02);
alt.init_struct(587,584,1);// set_point, input, mode, desired velocity=0
VelX.init_struct(0,0,1);
VelY.init_struct(0,0,-1);

///---------------------Set MOde and ARM----------------------//////

    ros::ServiceClient cl=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setmode;
    srv_setmode.request.base_mode=0;
    srv_setmode.request.custom_mode="GUIDED";
    if(cl.call(srv_setmode))
    {
         ROS_INFO("setmode send ok %d value:", srv_setmode.response.success);
    }
    else
    {
        ROS_ERROR("Failed Setmode");
        return -1;
    }
    sleep(2);
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
    sleep(2);
///---------------Takeoff----------------//////////////
  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 3;
  // srv_takeoff.request.latitude = -35.3632607;
  // srv_takeoff.request.longitude = 149.1652351;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = 0;
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(5);
////////-----------------////////////////////
geometry_msgs::TwistStamped msg_takeoff;
  // Do our own spin loop
  while (!g_request_shutdown)
  {
    // Do non-callback stuff
     double dect_time=(ros::Time::now()-aruco_time).toSec();
     if(dect_time<5)
     {
       ROS_ERROR("..!.!.!.Aruco Detectde.!.!.!..");
       throttle=alt_z.update_PID(alt.SetPoint,alt.input,alt.mode);
       roll=linear_vel*cos(theta) + vel_x.update_PID(VelX.SetPoint,VelX.input,VelX.mode);
       pitch= (-linear_vel)*sin(theta) + vel_y.update_PID(VelY.SetPoint,VelY.input,VelY.mode);
      if(throttle>MAX_VEL)throttle=MAX_VEL;
      else if(throttle<MIN_VEL)throttle=MIN_VEL;
      else if(roll>MAX_VEL)roll=MAX_VEL;
      else if(roll<MIN_VEL)roll=MIN_VEL;
      if(pitch>MAX_VEL)pitch=MAX_VEL;
      else if(pitch<MIN_VEL)pitch=MIN_VEL;
       msg_takeoff.twist.linear.z=throttle;
       msg_takeoff.twist.linear.x= roll;
       msg_takeoff.twist.linear.y=pitch;
      ROS_INFO_STREAM("Throttle: "<<throttle);
      ROS_INFO_STREAM("Roll: "<<roll);
      ROS_INFO_STREAM("Pitch: "<<pitch);
      if(((vel_x._error<0.05)&&(vel_x._error>(-0.05)))&&((vel_y._error<0.05)&&(vel_y._error>(-0.05))))
      { 
        ROS_WARN("!!!!!!!LANDED TRUE!!!!!!1");
        
      }
      vel_pub.publish(msg_takeoff);
     }
     else
     {
      ROS_ERROR("$$$......NOT DETECTED.....$$$$");
      throttle=alt_z.update_PID(alt.SetPoint,alt.input,alt.mode);
      msg_takeoff.twist.linear.z=throttle;
      msg_takeoff.twist.linear.x= BASE_VEL;
      msg_takeoff.twist.linear.y=BASE_VEL;
      ROS_INFO_STREAM("Throttle: "<<throttle);
      ROS_INFO_STREAM("Roll: "<<roll);
      ROS_INFO_STREAM("Pitch: "<<pitch);
      vel_pub.publish(msg_takeoff);
     }
    ros::spinOnce();
    rate.sleep();
  }

// Do pre-shutdown tasks
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