#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
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

#define MIN_VEL   -2
#define BASE_VEL  0
#define MAX_VEL   2


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
data VelX;
data VelY;
ros::Time aruco_time;
//error_data error;
error_data alt_error;
error_data VelX_error;
error_data VelY_error;
pid alt_gain, Velx_gain,VelY_gain;
bool already_searched=true; // change to falss if you have gps  coordinates for search
bool gps_input_flag=true;
bool landed=false;
/////////////////----------------------PID GAINS-------------------------------/////////////
double Kp=100; //100
double Ki=1.5; //1.5
double Kd=10; //10
//////////---------------Call BAck ---------------------///////////////
void gps_callback(const sensor_msgs::NavSatFix& msg)
{   if(gps_input_flag)
       alt.input=msg.altitude;
    ROS_INFO_STREAM("Hieght="<<alt.input<<std::endl);
}

void aruco_callback(const geometry_msgs::Vector3Stamped& pos)
{   if(!gps_input_flag)
        alt.input=pos.vector.z;
    aruco_time=pos.header.stamp;
    VelX.input=pos.vector.x;
    VelY.input=pos.vector.y;
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

    int update_pid(data &mode_input, error_data &mode_error, pid &mode_gain);
    void takeoff(double gps_alt);
    void search(void);
    void follow(double alt);
    void land(void);
    void initialize(error_data &test);
    geometry_msgs::TwistStamped msg;
    //-------------------INitialize struct---------------//////////
       // alt.SetPoint=588; //588// 1.4
       alt.input=584; //584
        alt.mode=1;
        VelX.SetPoint=0;
        VelX.mode=1;
        VelY.SetPoint=0;
        VelY.mode=-1;
   
        initialize(alt_error);
        initialize(VelX_error);
        initialize(VelY_error);
        // PID GAINS//
        alt_gain.kp=0.7;
        alt_gain.ki=0.0;
        alt_gain.kd=0.03;

        Velx_gain.kp=0.5;
        Velx_gain.ki=0.0;
        Velx_gain.kd=0.03;
        
        VelY_gain.kp=0.5;
        VelY_gain.ki=0.0;
        VelY_gain.kd=0.03;
            


    ///////////-------------------------////////////
    ros::NodeHandle nh;
    ros::Subscriber gps = nh.subscribe("/mavros/global_position/global", 1000, gps_callback);
    ros::Subscriber aruco = nh.subscribe("/aruco_single/position", 1000, aruco_callback);
    
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    ros::Rate rate(10);
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
  srv_takeoff.request.altitude = 1;
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
   alt.SetPoint=588;
   gps_input_flag=true;
   geometry_msgs::TwistStamped msg_takeoff;
   ROS_WARN("Taking Off!!");
   
  
   while(!g_request_shutdown)
   {
       throttle=update_pid(alt,alt_error,alt_gain);
        if(throttle>MAX_VEL)throttle=MAX_VEL;
        else if(throttle<MIN_VEL)throttle=MIN_VEL;
        msg_takeoff.twist.linear.z=throttle;
        msg_takeoff.twist.linear.x= BASE_VEL;
        msg_takeoff.twist.linear.y=BASE_VEL;
        vel_pub.publish(msg_takeoff);
        //ROS_INFO_STREAM("Error here=="<<alt_error.e<<std::endl);
        if((alt_error.e<0.9)&&(alt_error.e>-0.9))
         { ROS_ERROR("IF condition !!!");
            break;   
         }
        rate.sleep();
        ros::spinOnce();

   }
        
        msg_takeoff.twist.linear.z= BASE_VEL;
        msg_takeoff.twist.linear.x= BASE_VEL;
        msg_takeoff.twist.linear.y= BASE_VEL;
        vel_pub.publish(msg_takeoff);
        sleep(0.5);
        vel_pub.publish(msg_takeoff);
    ROS_INFO("..TakeOff Complete...");
///////////////------------------------------////////////////
initialize(alt_error);
sleep(5);
bool change=false;
int i=0;
gps_input_flag=true;
alt.SetPoint=588;
////////////----------------------------------////////////
    while(!g_request_shutdown)
    {   double dect_time=(ros::Time::now()-aruco_time).toSec();
        if(dect_time<5)
        {   
            if(change)
            {
                initialize(alt_error);
                initialize(VelX_error);
                initialize(VelY_error);
                change=false;
            }
          /*  gps_input_flag=true;
            alt.SetPoint=588;*/
            ROS_ERROR("Aruco Detected!!!");
            // Follow The marker .. 
                throttle=update_pid(alt,alt_error,alt_gain);
                roll=update_pid(VelX,VelX_error,Velx_gain);
                pitch=update_pid(VelY,VelY_error,VelY_gain);
                if(throttle>MAX_VEL)throttle=MAX_VEL;
                else if(throttle<MIN_VEL)throttle=MIN_VEL;
                /*if ((VelX.input<0.05)&&(VelX.input>(-0.05)))
                roll=BASE_VEL;*/
                else if(roll>MAX_VEL)roll=MAX_VEL;
                else if(roll<MIN_VEL)roll=MIN_VEL;    
                /*if ((VelY.input<0.05)&&(VelY.input>(-0.05)))
                pitch=BASE_VEL;*/
                if(pitch>MAX_VEL)pitch=MAX_VEL;
                else if(pitch<MIN_VEL)pitch=MIN_VEL;
                if((roll==BASE_VEL)&&(pitch==BASE_VEL))
                {   i++;
                    landed=true;
                    ROS_WARN("LANDED TRUE!! %d",i);
                  //  throttle=MIN_VEL;
                }
                ROS_INFO_STREAM("Throttle: "<<throttle);
                ROS_INFO_STREAM("Roll: "<<roll);
                ROS_INFO_STREAM("Pitch: "<<pitch);
           
                msg.twist.linear.z= throttle;
                msg.twist.linear.x= roll;
                msg.twist.linear.y= pitch;
                vel_pub.publish(msg);
            // if x and y error is less
                if(landed)
                {
                        
                    /*    srv.request.value = false;
                        if(arming_cl.call(srv))
                        {
                            ROS_INFO("DISARM send ok %d", srv.response.success);
                            //break
                            break;
                        }
                        else
                        {
                            ROS_ERROR("Failed arming or disarming");
                        }
                */
                }

        }
        else
        {
            ROS_ERROR("NOT DETEDCTED!!!1");
           
            if(!already_searched)
            {
                ///--------DO SEARCH------------------/////////
                
                // go to the gps Location///
                // set flag Already_serched == TRUE
            }
            else if ((already_searched))//&&(!landed)) // Already serched && not landed
            {
                
                //Gain a bit hieght ... fast
                 if(!change)
                {
                    initialize(alt_error);
                    initialize(VelX_error);
                    initialize(VelY_error);
                    change=true;  
                }
              /*  gps_input_flag=true;
                alt.SetPoint=588;*/
                throttle=update_pid(alt,alt_error,alt_gain);
                ROS_INFO_STREAM("Throttle: "<<throttle);
                ROS_INFO_STREAM("Roll: "<<roll);
                ROS_INFO_STREAM("Pitch: "<<pitch);
                msg.twist.linear.z= throttle;
                msg.twist.linear.x= BASE_VEL;
                msg.twist.linear.y= BASE_VEL;
                vel_pub.publish(msg);

            }
            
            

        }
        ros::spinOnce();
        rate.sleep();
    }
///////////////------------------------////////////////

/*    while(!g_request_shutdown)
    {

    throttle=1500+update_pid(alt,alt_error,alt_gain);
    roll=1500+update_pid(rollX,rollX_error,rollx_gain);
    pitch=1500+update_pid(pitchY,pitchY_error,pitchY_gain);
    if(throttle>MAX_VEL)throttle=MAX_VEL;
    else if(throttle<MIN_VEL)throttle=MIN_VEL;
    if ((rollX.input<0.09)&&(rollX.input>(-0.09)))
    roll=BASE_VEL;
    else if(roll>MAX_VEL)roll=MAX_VEL;
    else if(roll<MIN_VEL)roll=MIN_VEL;    
    if ((pitchY.input<0.09)&&(pitchY.input>(-0.09)))
    pitch=BASE_VEL;
    if(pitch>MAX_VEL)pitch=MAX_VEL;
    else if(pitch<MIN_VEL)pitch=MIN_VEL;
    ROS_INFO_STREAM("Throttle: "<<throttle);
    ROS_INFO_STREAM("Roll: "<<roll);
    ROS_INFO_STREAM("Pitch: "<<pitch);
    msg.channels[0] = roll;     //Roll
    msg.channels[1] = pitch;    //Pitch
    msg.channels[2] = throttle;   //Throttle
    msg.channels[3] = BASE_VEL;        //Yaw
    msg.channels[4] = 0;
    msg.channels[5] = 0;
    msg.channels[6] = 0;
    msg.channels[7] = 0;

    rc_pub.publish(msg);
    
    rate.sleep();
    
    ros::spinOnce();
        
    }*/
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



int update_pid(data &mode_input, error_data &mode_error, pid &mode_gain)
{
    int output;
    mode_error.e=(mode_input.mode)*(mode_input.SetPoint-mode_input.input);

    mode_error.e_diff=(mode_error.e-mode_error.e_old);
    //-----CLamping I-Term----//
    if(mode_error.e_sum<MAX_VEL)
    mode_error.e_sum+=(mode_gain.ki*mode_error.e);
    if(mode_error.e_sum<(MIN_VEL))
    mode_error.e_sum=MIN_VEL;
    ////-------------------/////
    double pid=mode_gain.kp*(mode_error.e)+(mode_error.e_sum)-(mode_gain.kd*mode_error.e_diff);
    output=static_cast<int>(pid);
    
    mode_error.e_old=mode_error.e;
    ROS_INFO_STREAM("OLD_ERROR"<<mode_error.e_old<<std::endl);
    return output;
}

void initialize( error_data &test)
{
    test.e=0;
    test.e_sum=0;
    test.e_diff=0;
    test.e_old=0;
}
/*void takeoff(double gps_alt)
{
   alt.SetPoint=gps_alt;
   
   mavros_msgs::OverrideRCIn msg_takeoff;
   ROS_WARN("Taking Off!!");
   while((alt_error.e>0.09)&&(alt_error.e<(-0.09))&&(!g_request_shutdown))
   {
       throttle=1500+update_pid(alt,alt_error,alt_gain);
        if(throttle>MAX_VEL)throttle=MAX_VEL;
        else if(throttle<MIN_VEL)throttle=MIN_VEL;
        msg_takeoff.channels[0] = BASE_VEL;     //Roll
        msg_takeoff.channels[1] = BASE_VEL;    //Pitch
        msg_takeoff.channels[2] = throttle;   //Throttle
        msg_takeoff.channels[3] = BASE_VEL;        //Yaw
        msg_takeoff.channels[4] = 0;
        msg_takeoff.channels[5] = 0;
        msg_takeoff.channels[6] = 0;
        msg_takeoff.channels[7] = 0;

        rc_pub.publish(msg_takeoff);
        ros::spinOnce();

   }
        msg_takeoff.channels[0] = BASE_VEL;     //Roll
        msg_takeoff.channels[1] = BASE_VEL;    //Pitch
        msg_takeoff.channels[2] = BASE_VEL;   //Throttle
        msg_takeoff.channels[3] = BASE_VEL;        //Yaw
        msg_takeoff.channels[4] = 0;
        msg_takeoff.channels[5] = 0;
        msg_takeoff.channels[6] = 0;
        msg_takeoff.channels[7] = 0;
        rc_pub.publish(msg_takeoff);
    ROS_INFO("..TakeOff Complete...");
}*/