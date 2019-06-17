#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <mavros_msgs/CommandTOL.h>
#include <time.h>
#include <ros/duration.h>
#include <cmath>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

const float POSITION_TOLERANCE = 0.1;
const float ON_TARGET_THRESHOLD = 20;

const float p = 0.2f;
const float i = 0.0;//005f;
const float d = 0.0f;


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  // ROS_INFO("I'm in state_cb!");
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr pose){
  // ROS_INFO("I'm in pose_cb!");
  current_pose = *pose;
}

float get_distance(double x1, double y1, double z1, double x2, double y2, double z2){
  return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

void go_to_position_pid(ros::NodeHandle* nh, float x, float y, float z){
  ros::Publisher set_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  geometry_msgs::TwistStamped vel;


  if (set_vel_pub)
  {

    printf("Going to position: %f, %f, %f\n", x, y, z);

    int onTargetCount = 0;
    float accumulatedErrorX = 0;
    float accumulatedErrorY = 0;
    float accumulatedErrorZ = 0;

    while(ros::ok()){
      float distance = get_distance(x,y,z,current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

      printf("Current position: %f,%f,%f. Distance: %f\n", 
                    current_pose.pose.position.x, 
                    current_pose.pose.position.y, 
                    current_pose.pose.position.z, 
                    distance);

      float errorX = x - current_pose.pose.position.x;
      float errorY = y - current_pose.pose.position.y;
      float errorZ = z - current_pose.pose.position.z;

      accumulatedErrorX += errorX;
      accumulatedErrorY += errorY;
      accumulatedErrorZ += errorZ;

      float velX = errorX*p + accumulatedErrorX*i;
      float velY = errorY*p + accumulatedErrorY*i;
      float velZ = errorZ*p + accumulatedErrorZ*i;

      vel.twist.linear.x = velX;
      vel.twist.linear.y = velY;
      vel.twist.linear.z = velZ;

      // if(x > current_pose.pose.position.x){
      //   vel.twist.linear.x = velocity;
      // }else{
      //   vel.twist.linear.x = -velocity;
      // }
      // if(y > current_pose.pose.position.y){
      //   vel.twist.linear.y = velocity;
      // }else{
      //   vel.twist.linear.y = -velocity;
      // }
      // if(z > current_pose.pose.position.z){
      //   vel.twist.linear.z = velocity;
      // }else{
      //   vel.twist.linear.z = -velocity;
      // }

      set_vel_pub.publish(vel);
      sleep(0.5);
      set_vel_pub.publish(vel);

      if(distance < POSITION_TOLERANCE){
        onTargetCount++;
      }

      if(onTargetCount > ON_TARGET_THRESHOLD){
        break;
      }

      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }

    vel.twist.linear.x = 0;
    vel.twist.linear.y = 0;
    vel.twist.linear.z = 0;
    set_vel_pub.publish(vel);
    sleep(0.5);
    set_vel_pub.publish(vel);

    ROS_INFO("Arrived at position");
  }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

   ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
    ROS_INFO("OFFBOARD enabled");
  else
  {
    ROS_INFO("unable to switch to offboard");
    return -1;
  }

  sleep(2);

  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
  else
  {
    ROS_ERROR("Failed arming/disarming");
    //return -1;
  }

  sleep(2);


  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 2;
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

  sleep(10);

  ROS_INFO("Going to position 1");
  go_to_position_pid(&nh, 2,2,2);

  sleep(5);

  
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


