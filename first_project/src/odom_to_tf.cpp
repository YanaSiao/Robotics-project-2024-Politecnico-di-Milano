#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <tf2/transform_datatypes.h>

class Odom_to_tf{

private:
    ros::NodeHandle n;

    tf::TransformBroadcaster br;
    ros::Subscriber sub;
    std_msgs::String root;
    std_msgs::String child;

public:

    //Constructor in which the node subscribes to a topic (needs to be remapped in launch file)
    Odom_to_tf(){
        sub = n.subscribe("input_odom", 1000, &Odom_to_tf::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        //Create the Transform object
        geometry_msgs::TransformStamped transform;

        //transform.setOrigin( tf::Vector3(msg->x, msg->y, 0) );
        //tf::Quaternion q;
        //q.setRPY(0, 0, msg->theta);
        //transform.setRotation(q);

        n.getParam("root_frame",root.data);
        std::string param_name = ros::this_node::getName() + "/child_frame";
        n.getParam(param_name,child.data);

        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = root.data;
        transform.child_frame_id = child.data;
        // Extract pose from odometry
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;

        // Extract orientation from odometry
        transform.transform.rotation.w = msg->pose.pose.orientation.w;
        transform.transform.rotation.z = msg->pose.pose.orientation.z;

        ROS_INFO("Dati presi dal topic tf :%s, X %f - Y %f - Z %f", child.data.c_str(),msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
        br.sendTransform(transform);

    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_to_tf");

    Odom_to_tf my_tf_sub_bub;
    ros::spin();

    return 0;
}