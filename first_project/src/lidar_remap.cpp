#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/lidar_frameIDConfig.h>

class Lidar_remap {

private:
    ros::NodeHandle n;

    ros::Subscriber lidar_sub;
    ros::Publisher lidar_pub;
    std::string frame = "wheel_odom";

public:

    void configCallback(first_project::lidar_frameIDConfig &config, uint32_t level) {
        frame = config.frame;

        // Update the TF frame ID based on the selected value
        ROS_INFO("Reconfigured Frame: %s", config.frame.c_str());
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 remapped_msg = *msg;
        remapped_msg.header.frame_id = frame;

        lidar_pub.publish(remapped_msg);
        ROS_INFO("Aligned data : %s,", remapped_msg.header.frame_id.c_str());
    }

    void init(){
        lidar_sub = n.subscribe("os_cloud_node/points", 1000, &Lidar_remap::lidarCallback, this);
        lidar_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);

        dynamic_reconfigure::Server<first_project::lidar_frameIDConfig> server;
        dynamic_reconfigure::Server<first_project::lidar_frameIDConfig>::CallbackType f;

        f = boost::bind(&Lidar_remap::configCallback, this, _1, _2);
        server.setCallback(f);

        ros::spin();
    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_remap");

    Lidar_remap lidar_remap;

    lidar_remap.init();

    return 0;
}
