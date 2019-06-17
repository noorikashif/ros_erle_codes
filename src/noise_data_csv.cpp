#include <iostream>

#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <random>

#include <fstream>

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
////////////---------------------------///////////////

//using namespace std;

// create an ofstream for the file output (see the link on streams for
// more info)
std::ofstream error_file;
std::ofstream data_file;

//ofstream fs;
std_msgs::Float32 error_data,range_data;
double ax,ay,az=0;
const double mean = 0.0;
const double stddev = 1;
std::default_random_engine generator;
std::normal_distribution<double> dist(mean, stddev);

// create a name for the file output
void sonar_callback(const sensor_msgs::Range& msg)
{
    range_data.data=msg.range;
    ROS_INFO_STREAM("SOnar Call back"<<range_data.data<<std::endl);
    data_file<<msg.range<<std::endl;
    error_data.data= range_data.data+dist(generator);
    error_file<<error_data.data<<","<<ax<<","<<ay<<","<<az<<std::endl;
   
}
void acc_Callback(const sensor_msgs::Imu& acc_msg)
{
    ax= acc_msg.linear_acceleration.x;
    ay= acc_msg.linear_acceleration.y;
    az=acc_msg.linear_acceleration.z;
    ROS_INFO_STREAM("ACC Call back"<<az<<std::endl);
}
int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "noise_data",ros::init_options::NoSigintHandler);
    signal(SIGINT,mySigIntHandler);
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown",shutdownCallback);
    
    
    ros::NodeHandle nh;
    ros::Subscriber sonar = nh.subscribe("/sonar_down", 1000, sonar_callback);
    ros::Subscriber acc = nh.subscribe("/mavros/imu/data", 1000, acc_Callback);
    
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/error_sonar", 10);

    //ros::Rate rate(10); 

    
    error_file.open("/home/kashif/error_sonar_data.csv");
    data_file.open("/home/kashif/sonar_data.csv");
    error_file<<"Error_data"<<","<<"Ax"<<","<<"Ay"<<","<<"Az"<<std::endl;
    data_file<<"Original Data"<<std::endl;
    

   
    while (!g_request_shutdown)
    {
        /* code for loop body */
   
     pub.publish(error_data);
     //rate.sleep();
     ros::spinOnce();
    }
    
    ROS_INFO("Closing files now");
    error_file.close();
    data_file.close();
    ROS_INFO("CLosed Files!");
    return 0;
}