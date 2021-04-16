/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <fstream> //file save
#include <sys/stat.h>
#include <ctime>

#include <std_srvs/Empty.h>

std::string save_folder;
float map_res;
ros::Publisher pub_;

typedef pcl::PCLPointCloud2 PointCloudT;

PointCloudT::Ptr cloud_map = pcl::make_shared<PointCloudT>();

//Callback da nuvem
void callback(const sensor_msgs::PointCloud2::ConstPtr &pc)
{
    ROS_INFO_ONCE("Got Scan!");

    PointCloudT::Ptr pc2(new PointCloudT);
    pcl_conversions::toPCL(*pc, *pc2);

    pcl::VoxelGrid<PointCloudT> voxel;
    voxel.setInputCloud(pc2);
    voxel.setLeafSize(map_res, map_res, map_res);
    voxel.filter(*pc2);

    pcl::concatenate(*cloud_map, *pc2, *cloud_map);
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

    if (pcl::io::savePCDFile(str_pcd, *cloud_map) != 0)
    {
    }

    ROS_INFO("Done!");
    return true;
}

bool clear_cloud(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Clearing cloud...");
    return true;
}

//ler inteiro

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_saver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string input_cloud_node = nh.resolveName("velodyne_cloud_registered");

    ros::Subscriber cloud_sub = nh.subscribe(input_cloud_node, 1, callback);

    if (!private_nh.getParam("save_folder", save_folder))
    {
        char *home_folder = getenv("HOME");
        save_folder = home_folder;
    }

    private_nh.param<float>("map_res", map_res, 0.1f);

    ros::ServiceServer save_service = private_nh.advertiseService("save", save_cloud);

    ros::ServiceServer clear_service = private_nh.advertiseService("clear", clear_cloud);

    pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);

    ROS_WARN("Map Res: %f", map_res);
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
