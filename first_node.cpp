#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#define SEMI_MAJOR_AXIS 6378137
#define SEMI_MINOR_AXIS 6356752

float64 gpsToEcef[](float64 lat, float64 lon, float64 alt){
  float64 X, Y, Z;
  float64 N;
  float64 esq = 1- pow(b, 2)/pow(a,2);

  N = a / sqrt(1- esq*pow(sin(lat),2));
  X = cos(lat)*cos(lon)*(alt + N);
  Y = cos(lat)*sin(lon)*(alt + N);
  Z = sin(lat)*(alt + N*(1-esq));  //1-(1+c) = c = 0.9933055
  float6464 ecef_point[3] = {X,Y,Z};

  return ecef_point;
}

float64 ecefToNed[](float64 ref_point [], float64 ecef_point [],float64 lat_r, float64 lon_r, float64 alt_r){
  float64 A [3][3] = {{-sin(lon_zero), cos(lon_r), 0},{-sin(lat_r)*cos(lon_r), -sin(lat_r)*sin(lon_r), cos(lat_r)},{cos(lat_r)*cos(lon_r), cos(lat_r)*sin(lon_r), sin(lat_r)}};
  float64 B [3][1] = {{ecef_point[0] - ref_point[0]},{ecef_point[1]-ref_point[1]},{ecef_point[2]-ref_point[2]}};
  float64 C [3] = {{0}, {0}, {0}};
  for( int i = 0; i < 3; i++) {
    for(int j =0; j<3; j++) {
      C[i] += A[i][j]*B[j][1];
    }
  }
  return C;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_to_odom");
  ros::NodeHandle nh;

  // Advertise odometry topic
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);

  // Advertise odometry topic
  ros::Publisher odom_pub = nh.advertise<geometry_msgs::Odometry>("gps_odom", 10);
 // Set initial position (replace with values from first GPS message)
  sensor_msgs::NavSatFix gps;
  double lat_first = gps.latitude;
  double lon_first = gps.longitude;
  double alt_first = gps.altitude;

  double lat = lat_first;
  double lon = lon_first;
  double alt = alt_first;
  float64 ref_point[] = gpsToEcef(lat, lon, alt);
  float64 prev_ned_point[] = ecefToNed(ref_point, lat_first, lon_first, alt_first);

  // Main loop
  ros::Rate rate(10); // Set publishing rate

  while (ros::ok()) {
    // Convert GPS data to ECEF.
    float64 ecef_point [] = gpsToEcef(lat, lon, alt);

    // Calculate relative position in NED frame.
    float64 ned_point []= ecefToNed(ref_point, ecef_point, lat_first, lon_first, alt_first);

    // Calculate displacement for heading estimation
    float64 displacement[] = ned_point - prev_ned_point;
    prev_ned_point = ned_point;  // Update previous position for next iteration

    // Estimate heading (assuming 2D)
    // atan2 is a four-quadrant inverse tangent in rad
    double orientation = std::atan2(displacement[1], displacement[0]);
    double heading_deg = orientation * 180.0 / M_PI;

    // Create odometry message
    nav_msgs::Odometry odom_msg;

    // Set position in NED frame
    odom_msg.pose.pose.position.x = ned_point[0];
    odom_msg.pose.pose.position.y = ned_point[1];
    odom_msg.pose.pose.position.z = ned_point[2];

    // Optional: Include estimated heading in odometry message
    odom_msg.pose.pose.orientation.w = cos(orientation );  // Assuming no roll
    odom_msg.pose.pose.orientation.z = sin(orientation );

    // Publish odometry message
    odom_pub.publish(odom_msg);

    // Create TF transform (optional?)
    nav_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();

    transform.transform.translation.x = ned_point[0];
    transform.transform.translation.y = ned_point[1];
    transform.transform.translation.z = ned_point[2];

    transform.transform.rotation.w = cos(orientation);  // Assuming no roll

    rate.sleep();

//  receiving new GPS data
   lat = gps.latitude;
   lon = gps.longitude;
   alt = gps.altitude;
  }
    return 0;
}