#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/dynamic_reconfigureConfig.h>

class Lidar_remap {

private:
    ros::NodeHandle n;

    ros::Subscriber lidar_sub;
    ros::Publisher lidar_pub;
    ros::string frame = "wheel_odom";

public:

    void callback(first_project::dynamic_reconfigureConfig &config, uint32_t level) {
        frame = config.frame;

        // Update the TF frame ID based on the selected value

        ROS_INFO("Reconfigured Frame: %s", config.frame.c_str());
    }

    void configCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        msg.points.header = frame;

        odom_pub.publish(msg);
        ROS_INFO("Aligned data : %s,", msg.points.header.c_str());
    }

    void init(){
        lidar_sub = n.subscribe("os_cloud_node/points", 1000, &lidar_remap::configCallback, this);
        odom_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud_remapped", 1000);

        dynamic_reconfigure::Server<first_project::dynamic_reconfigureConfig> server;
        dynamic_reconfigure::Server<first_project::dynamic_reconfigureConfig>::CallbackType f;

        f = boost::bind(&callback, _1, _2);
        server.setCallback(f);

        ros::spin();
    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_remap");

    Lidar_remap lidar_sub;

    lidar_sub.init();

    return 0;
}
