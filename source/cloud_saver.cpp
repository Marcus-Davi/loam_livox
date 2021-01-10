/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <fstream> //file save
#include <sys/stat.h>
#include <ctime>

#include <std_srvs/Empty.h>

std::string save_folder;
ros::Publisher pub_;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

PointCloudT::Ptr cloud_map = pcl::make_shared<PointCloudT>();

//Callback da nuvem
void callback(const sensor_msgs::PointCloud2::ConstPtr &pc)
{
    ROS_INFO_ONCE("Got Scan!");

    static PointCloudT::Ptr cloud_scan = pcl::make_shared<PointCloudT>();
    pcl::fromROSMsg(*pc, *cloud_scan);

    *cloud_map += *cloud_scan;

    sensor_msgs::PointCloud2::Ptr cloud_map_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud_map, *cloud_map_msg);
    // cloud_map_msg->header.stamp = ros::Time::now();
    cloud_map_msg->header.frame_id = "map";
    pub_.publish(cloud_map_msg);
}

bool save_cloud(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Saving cloud...");
    ros::Time time = ros::Time::now();

    // std::time_t std_time;
    // std_time = time.sec;
    // char *time_date = std::ctime(&std_time);

    std::string str_path = save_folder + "/slam_cloud_";
    std::string str_time = std::to_string(time.sec);
    std::string str_pcd = str_path + str_time + ".pcd";

    if(cloud_map->size())
    pcl::io::savePCDFileBinary(str_pcd, *cloud_map); //Mais Eficiente ?
    else
    {
        ROS_ERROR("No points to be saved.");
    }
    

    ROS_INFO("Done!");
    return true;
}

bool clear_cloud(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Clearing cloud...");
    cloud_map->clear();
    ROS_INFO("Done!");
    return true;
}

//ler inteiro

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_saver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string input_cloud_node = nh.resolveName("cloud");

    ros::Subscriber cloud_sub = nh.subscribe(input_cloud_node, 1, callback);

    if( !private_nh.getParam("save_folder",save_folder) ){
        char* home_folder = getenv("HOME");
        save_folder = home_folder;
    }

    ros::ServiceServer save_service = private_nh.advertiseService("save", save_cloud);

    ros::ServiceServer clear_service = private_nh.advertiseService("clear", clear_cloud);

    pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);

    ROS_WARN("Subbed Cloud topic -> %s", input_cloud_node.c_str());
    ROS_WARN("Save folder -> %s", save_folder.c_str());

    ros::spin();

    return 0;
}

// void ResolveDir(){
// struct stat statbuff;
// bool isDir = 0;
// char* home_path = getenv("HOME");
// string pontos_path = std::string(home_path) + "/Pontos/";
// if(stat(pontos_path.c_str(),&statbuff) == -1) {
// 	mkdir(pontos_path.c_str(),0755);
// 	}

// }
