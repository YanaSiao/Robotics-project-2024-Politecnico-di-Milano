
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class MarkerPublisher {

private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    tf::TransformListener listener;

    double marker_size_x, marker_size_y, marker_size_z;


public:
    MarkerPublisher() {
        ros::NodeHandle private_nh("~");
        private_nh.param("marker_size_x", marker_size_x, 0.612);
        private_nh.param("marker_size_y", marker_size_y, 0.580);
        private_nh.param("marker_size_z", marker_size_z, 0.245);

        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    }

    void publish_marker() {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = transform.getOrigin().x();
        marker.pose.position.y = transform.getOrigin().y();
        marker.pose.position.z = transform.getOrigin().z();
        marker.pose.orientation.x = transform.getRotation().x();
        marker.pose.orientation.y = transform.getRotation().y();
        marker.pose.orientation.z = transform.getRotation().z();
        marker.pose.orientation.w = transform.getRotation().w();
        marker.scale.x = 0.612;  // Dimensione del cubo lungo l'asse X
        marker.scale.y = 0.580;  // Dimensione del cubo lungo l'asse Y
        marker.scale.z = 0.245;  // Dimensione del cubo lungo l'asse Z
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker_pub.publish(marker);
    }

    void init() {
        ros::Rate rate(10);  // Frequenza di pubblicazione 10 Hz
        ros::Duration(2.0).sleep();  // Aggiungi un ritardo di 2 secondi

        while (ros::ok()) {
            publish_marker();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "marker");

    MarkerPublisher marker_publisher;
    marker_publisher.init();
    ROS_INFO("Marker publisher initialized!");

    return 0;
}