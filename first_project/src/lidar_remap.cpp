#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>

class Lidar_remap {

private:
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ros::Subscriber lidar_sub;

    tf_frame_id:
        name: LiDAR Reference Frame
        type: string
        default: "wheel_odom"
        enum : ["wheel_odom", "gps_odom"]

public:

    Lidar_remap() {
        Lidar_sub = n.subscribe("os_cloud_node/points pointcloud", 1000, &lidar_remap::callback, this);
    }

    void configCallback(first_project::header_config &config, uint32_t level) {

        // Update the TF frame ID based on the selected value
        your_tf_broadcaster.set_frame_id(config.tf_frame_id);

        ROS_INFO("Reconfigured: TF Frame: %s", config.tf_frame_id.c_str());
    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_remap");

    Lidar_remap lidar_sub;

    dynamic_reconfigure::Server<first_project::header_config> server;
    server.setCallback(configCallback);

    ros::spin();


    return 0;
}
