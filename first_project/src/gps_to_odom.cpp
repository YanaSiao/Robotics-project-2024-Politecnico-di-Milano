#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#define SEMI_MAJOR_AXIS 6378137
#define SEMI_MINOR_AXIS 6356752

class Gps_to_odom{

private:
    ros::Publisher odom_pub;
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;

    double lat_first = 0.0;
    double lon_first = 0.0;
    double alt_first = 0.0;

    std::vector<double> ref_point;
    std::vector<double> prev_ned_point;

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;

    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& message){
        //in ogni caso devo settare le nuove variabili, anche quando siamo alla prima volta!
        lat = message -> latitude;
        lon = message -> longitude;
        alt = message -> altitude;
        ROS_INFO("Dati base: %f %f %f", lat,lon,alt);
        std::vector<double> ecef_point = gpsToEcef(lat,lon,alt);
        //ROS_INFO("Dati ecef: %f %f %f", ecef_point[0],ecef_point[1],ecef_point[2]);
        std::vector<double> ned_point = ecefToEnu(ref_point,ecef_point);
        // Calculate displacement for heading estimation

        std::vector<double> displacement(3);
        std::set_difference(ned_point.begin(), ned_point.end(), prev_ned_point.begin(), prev_ned_point.end(),displacement.begin());
        std::copy(ned_point.begin(),ned_point.end(), prev_ned_point.begin());  // Update previous position for next iteration

        // Estimate heading (assuming 2D)
        double orientation = atan2(displacement[1], displacement[0]);

        publish_odom_message(ned_point,orientation);
    }

public:
    std::vector<double> gpsToEcef(double lat_deg, double lon_deg, double altit){
        double X, Y, Z;
        double N;
        double esq = 1- pow(SEMI_MINOR_AXIS, 2)/pow(SEMI_MAJOR_AXIS,2);

        double lat_rad = lat_deg * M_PI / 180;
        double lon_rad = lon_deg * M_PI / 180;
        //ROS_INFO("Dati base radiant: %f %f %f", lat_rad,lon_rad,altit);
        N = SEMI_MAJOR_AXIS / sqrt(1- esq*pow(sin(lat_rad),2));

        X = cos(lat_rad)*cos(lon_rad)*(altit + N);
        Y = cos(lat_rad)*sin(lon_rad)*(altit + N);
        Z = sin(lat_rad)*(altit + N*(1-esq));
        std::vector<double> ecef_point = {X,Y,Z};

        return ecef_point;
    }

    std::vector<double> ecefToEnu(std::vector<double> ref_point, std::vector<double> ecef_point){
        std::vector<std::vector<double>> A = {
                {-sin(ref_point[1]), cos(ref_point[1]), 0},
                {-sin(ref_point[0]) * cos(ref_point[1]), -sin(ref_point[0]) * sin(ref_point[1] ), cos(ref_point[0])},
                {cos(ref_point[0]) * cos(ref_point[1]), cos(ref_point[0]) * sin(ref_point[1] ), sin(ref_point[0])}
        };
        std::vector<std::vector<double>> B= {
                {ecef_point[0] - ref_point[0]},
                {ecef_point[1] - ref_point[1]},
                {ecef_point[2] - ref_point[2]}
        };
        std::vector<double> C= {0, 0, 0};
        for( int i = 0; i < 3; i++) {
            for(int j =0; j<3; j++) {
                C[i] += A[i][j]*B[j][0];
            }
        }
        return C;
    }

    void publish_odom_message(std::vector<double> enu_point,double orientation){

        nav_msgs::Odometry odom_msg;

        // Set position in NED frame
        odom_msg.pose.pose.position.x = enu_point[0];
        odom_msg.pose.pose.position.y = enu_point[1];
        odom_msg.pose.pose.position.z = enu_point[2];

        // Optional: Include estimated heading in odometry message (might be inaccurate)
        odom_msg.pose.pose.orientation.w = cos(orientation);  // Assuming no roll
        odom_msg.pose.pose.orientation.z = sin(orientation);

        // Publish odometry message
        odom_pub.publish(odom_msg);

        ROS_INFO("odom trasformed message has been published X: %f, Y: %f, Z: %f", odom_msg.pose.pose.position.x,
                 odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z);
    }

    void init(){
        prev_ned_point = {0,0,0};

        nh.getParam("lat_r", lat_first);
        nh.getParam("lon_r", lon_first);
        nh.getParam("alt_r", alt_first);
        ref_point = gpsToEcef(lat_first,lon_first,alt_first);

        gps_sub = nh.subscribe("fix", 1000, &Gps_to_odom::fixCallback,this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1000);
        ros::spin(); //fa il check sulle chiamate ai callback

    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");

    Gps_to_odom odom;
    odom.init();

    return 0;
}