//gps_to_odom

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>

// Define constants (replace with GPS initial data)
double initial_lat = 0.0;
double initial_lon = 0.0;
double initial_alt = 0.0;

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_to_odom");
  ros::NodeHandle nh;

  // Set initial position (replace with values from first GPS message)
  double lat_zero = initial_lat;
  double lon_zero = initial_lon;
  double alt_zero = initial_alt;

  // Advertise odometry topic
  ros::Publisher odom_pub = nh.advertise<geometry_msgs::Odometry>("gps_odom", 10);

  // Create TF broadcaster
  tf2_ros::TransformBroadcaster br;
  // Store previous position for heading estimation
  Eigen::Vector3d prev_ned_point(0.0, 0.0, 0.0);

  // Main loop
  ros::Rate rate(10); // Set publishing rate (10 Hz in this example)
  while (ros::ok()) {
    // Simulate receiving new GPS data (replace with actual data)
    double lat = initial_lat + 0.1; // Replace with latitude data
    double lon = initial_lon + 0.2; // Replace with longitude data
    double alt = initial_alt + 0.3; // Replace with altitude data

    // Convert GPS data to ECEF
    Eigen::Vector3d ecef_point = some_ecef_to_ned_library::gpsToEcef(lat, lon, alt);

    // Calculate relative position in NED frame
    Eigen::Vector3d ned_point = some_ecef_to_ned_library::ecefToNed(ecef_point, lat_zero, lon_zero, alt_zero);

    // Calculate displacement for heading estimation
    Eigen::Vector3d displacement = ned_point - prev_ned_point;
    prev_ned_point = ned_point;  // Update previous position for next iteration

    // Estimate heading (assuming flat terrain)
    double orientation = atan2(displacement[1], displacement[0]);
    double heading_deg = orientation * 180.0 / M_PI;

    // Create odometry message
    geometry_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "gps_link";
    odom_msg.child_frame_id = "base_link";

    // Set position in NED frame
    odom_msg.pose.pose.position.x = ned_point[0];
    odom_msg.pose.pose.position.y = ned_point[1];
    odom_msg.pose.pose.position.z = ned_point[2];

    // Optional: Include estimated heading in odometry message (might be inaccurate)
    odom_msg.pose.pose.orientation.w = cos(orientation / 2.0);  // Assuming no roll or pitch
    odom_msg.pose.pose.orientation.z = sin(orientation / 2.0);

    // Publish odometry message
    odom_pub.publish(odom_msg);

    // Create TF transform (optional)
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "gps_link";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = ned_point[0];
    transform.transform.translation.y = ned_point[1];
    transform.transform.translation.z = ned_point[2];
    transform.transform.rotation.w = cos(orientation / 2.0);  // Assuming no roll or pitch

    // Broadcast TF transform (optional)
    br.sendTransform(transform);

    rate.sleep();
  }
    return 0;
}
