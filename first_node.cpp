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

    bool first_time = true;

    double lat_first = 0.0;
    double lon_first = 0.0;
    double alt_first = 0.0;

    std::vector<double> ref_point; //yana dimmi se ho fatto bene, cioè qua dovresti fare il punto di riferimento
    //(tra l'altro una volta sola) e quindi gli passi xxx_first giusto?
    // tra l'altro non so se posso metterlo qui LOL però alla fine va fatto una volta sol
    std::vector<double> prev_ned_point;

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;

    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& message){
        if(first_time){
            lat_first  = message -> latitude;
            lon_first  = message -> longitude;
            alt_first = message -> altitude;
            this -> first_time = false;
        }

        //in ogni caso devo settare le nuove variabili, anche quando siamo alla prima volta!
        lat = message -> latitude;
        lon = message -> longitude;
        alt = message -> altitude;
        std::vector<double> ecef_point = gpsToEcef(lat,lon,alt);

        std::vector<double> ned_point = ecefToNed(ref_point,ecef_point);
        // Calculate displacement for heading estimation

        std::vector<double> displacement(3);
        std::set_difference(ned_point.begin(), ned_point.end(), prev_ned_point.begin(), prev_ned_point.end(),displacement.begin());
        std::copy(ned_point.begin(),ned_point.end(), prev_ned_point.begin());  // Update previous position for next iteration

        // Estimate heading (assuming 2D)
        double orientation = atan2(displacement[1], displacement[0]);
        //double heading_deg = orientation * 180.0 / M_PI;

        publish_odom_message(ned_point,orientation);
    }

public:
    std::vector<double> gpsToEcef(double lat_deg, double lon_deg, double alt_deg){
        double X, Y, Z;
        double N;
        double esq = 1- pow(SEMI_MINOR_AXIS, 2)/pow(SEMI_MAJOR_AXIS,2);

        double lat_rad = lat_deg * M_PI / 180;
        double lon_rad = lon_deg * M_PI / 180;

        N = SEMI_MAJOR_AXIS / sqrt(1- esq*pow(sin(lat),2));

        X = cos(lat_rad)*cos(lon_rad)*(alt_deg + N);
        Y = cos(lat_rad)*sin(lon_rad)*(alt_deg + N);
        Z = sin(lat_rad)*(alt_deg + N*(1-esq));  //1-(1+c) = c = 0.9933055
        std::vector<double> ecef_point = {X,Y,Z};

        return ecef_point;
    }

    void publish_odom_message(std::vector<double> ned_point,double orientation){

        nav_msgs::Odometry odom_msg;

        // Set position in NED frame
        odom_msg.pose.pose.position.x = ned_point[0];
        odom_msg.pose.pose.position.y = ned_point[1];
        odom_msg.pose.pose.position.z = ned_point[2];

        // Optional: Include estimated heading in odometry message (might be inaccurate)
        odom_msg.pose.pose.orientation.w = cos(orientation);  // Assuming no roll
        odom_msg.pose.pose.orientation.z = sin(orientation);

        // Publish odometry message
        odom_pub.publish(odom_msg);

        //ROS_INFO("odom message has been published: %s", odom_msg.pose.pose.position.x.c_str()); //non so come trasformarlo in stringa LOL
    }

    void init(){
        ref_point = gpsToEcef(lat_first, lon_first, alt_first); //inizializzo l'attributo della classe
        prev_ned_point = {0,0,0};

        odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);
        gps_sub = nh.subscribe("/fix", 10, &Gps_to_odom::fixCallback,this);

        ros::spin(); //fa il check sulle chiamate ai callback
    }

    std::vector<double> ecefToNed(std::vector<double> ref_point, std::vector<double> ecef_point){
        std::vector<std::vector<double>> A = {
                {-sin(lon_first * M_PI / 180), cos(lon_first * M_PI / 180), 0},
                {-sin(lat_first * M_PI / 180) * cos(lon_first * M_PI / 180), -sin(lat_first * M_PI / 180) * sin(lon_first * M_PI / 180), cos(lat_first * M_PI / 180)},
                {cos(lat_first * M_PI / 180) * cos(lon_first * M_PI / 180), cos(lat_first * M_PI / 180) * sin(lon_first * M_PI / 180), sin(lat_first * M_PI / 180)}
        };
        std::vector<std::vector<double>> B= {
                {ecef_point[0] - ref_point[0]},
                {ecef_point[1]-ref_point[1]},
                {ecef_point[2]-ref_point[2]}
        };
        std::vector<double> C= {{0}, {0}, {0}}; //questo non ho capito se volevi fare un array, perchè così mi sa che hai fatto una matrice strana (?)
        for( int i = 0; i < 3; i++) {
            for(int j =0; j<3; j++) {
                C[i] += A[i][j]*B[j][0];
            }
        }
        return C;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");

    Gps_to_odom odom;

    odom.init();

    // Main loop
    ros::Rate rate(10); // Set publishing rate
    while (ros::ok()) {

        rate.sleep();
    }
    return 0;
}


/*
//SECONDO FILE

// Create TF transform (optional?) questo non so se va fatto qua
    nav_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();

    transform.header.frame_id = "gps_link";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = ned_point[0];
    transform.transform.translation.y = ned_point[1];
    transform.transform.translation.z = ned_point[2];

    transform.transform.rotation.w = cos(orientation);  // Assuming no roll or pitch

  // Create TF broadcaster
  tf2_ros::TransformBroadcaster br;
  // Store previous position for heading estimation
  Eigen::Vector3d prev_ned_point(0.0, 0.0, 0.0);

 // Broadcast TF transform (optional?)
    br.sendTransform(transform);
*/