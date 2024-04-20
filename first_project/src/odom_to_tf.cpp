#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transformation_broadcaster.h>

class Odom_to_tf{

private:
    ros::NodeHandle n;

    tf::TransformBroadcaster broadcaster;
    ros::Subscriber sub;

public:

    //Constructor in which the node subscribes to a topic (needs to be remapped in launch file)
    Odom_to_tf(){
        sub = n.subscribe("input_odom", 10, &Odom_to_tf::callback, this);
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        //Create the Transform object
        tf::Transform transform;

        //Populate the object with odometry data


        //Publish the transformation using broadcaster
        //Consider this StampedTransform constructor:
        //  tf::StampedTransform::StampedTransform( const tf::Transform & input,
        //                                          const ros::Time & timestamp,
        //                                          const std::string & frame_id,
        //                                          const std::string &	child_frame_id)
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "root_frame", "child_frame")); //P.S. "root_frame" and "child_frame" have to be remapped
    }

};

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_to_tf");

    ros::spin();

    return 0;
}