#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    std::vector<double> prev_enu_point;

    double orientation;
    double prev_orientation = 0.0; // Initialize with any default value

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;

public:
    void init(){
        prev_enu_point= {0, 0, 0};

        nh.getParam("lat_r", lat_first);
        nh.getParam("lon_r", lon_first);
        nh.getParam("alt_r", alt_first);

        ref_point = gpsToEcef(lat_first,lon_first,alt_first);

        gps_sub = nh.subscribe("fix", 1000, &Gps_to_odom::fixCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1000);

        ros::spin(); //fa il check sulle chiamate ai callback
    }

    std::vector<double> rotate2D(std::vector<double> xy, double angle_degrees) {
        // Convert angle to radians
        double angle_radians = angle_degrees * M_PI / 180.0;

        // Rotation matrix
        std::vector<std::vector<double>> R = {{cos(angle_radians), -sin(angle_radians)},
                                              {sin(angle_radians), cos(angle_radians)}};

        // Rotate the vector
        std::vector<double> rotated_xy={0,0,0};

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                rotated_xy[i] += R[i][j] * xy[j];
            }
        }

        return rotated_xy;
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);  // Roll, pitch, yaw (in radians)

        geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat);

        return quat_msg;
    }

    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& message){
        //in ogni caso devo settare le nuove variabili, anche quando siamo alla prima volta!
        lat = message -> latitude;
        lon = message -> longitude;
        alt = message -> altitude;
        ROS_INFO("Dati base: %f %f %f", lat,lon,alt);

        std::vector<double> ecef_point = gpsToEcef(lat, lon, alt);
        //ROS_INFO("Dati ecef: %f %f %f", ecef_point[0],ecef_point[1],ecef_point[2]);

        std::vector<double> enu_point = ecefToEnu(ref_point,ecef_point);

        enu_point = rotate2D(enu_point,130);

        // Calculate displacement for heading estimation

        std::vector<double> displacement(2);

        // Calculate displacement (assuming NED coordinates)
        displacement[0] = enu_point[0] - prev_enu_point[0]; // Easting difference
        displacement[1] = enu_point[1] - prev_enu_point[1]; // Northing difference

        // Handle zero displacement to avoid division by zero
        if (displacement[0] == 0) {
            orientation = prev_orientation;
        } else {
            // Calculate orientation using arctangent (atan2 for signed angle)
            orientation = atan2(displacement[1], displacement[0]) /*+ 129 * M_PI / 180*/;
        }

        // Update previous position for next iteration (after orientation calculation)
        prev_enu_point[0] = enu_point[0];
        prev_enu_point[1] = enu_point[1];

        orientation = normalizeAngle(orientation);

        prev_orientation = orientation;

        publish_odom_message(enu_point,orientation,message->header.stamp);
    }

    std::vector<double> gpsToEcef(double lat_deg, double lon_deg, double altit){
        double X, Y, Z;
        double N;
        double esq = 1 - pow(SEMI_MINOR_AXIS, 2)/pow(SEMI_MAJOR_AXIS,2);

        double lat_rad = lat_deg * M_PI / 180;
        double lon_rad = lon_deg * M_PI / 180;
        N = SEMI_MAJOR_AXIS / sqrt(1 - esq * pow(sin(lat_rad),2));

        X = cos(lat_rad) * cos(lon_rad) * (altit + N);
        Y = cos(lat_rad) * sin(lon_rad) * (altit + N);
        Z = sin(lat_rad) * (altit + N * (1-esq));
        std::vector<double> ecef_point = {X,Y,Z};

        return ecef_point;
    }

    std::vector<double> ecefToEnu(std::vector<double> ref_point, std::vector<double> ecef_point){
        double lat_first_rad = lat_first * M_PI / 180;
        double lon_first_rad = lon_first * M_PI / 180;

        std::vector<std::vector<double>> A = {
                {-sin(lon_first_rad),                      cos(lon_first_rad),                       0                  },
                {-sin(lat_first_rad) * cos(lon_first_rad), -sin(lat_first_rad) * sin(lon_first_rad), cos(lat_first_rad) },
                {cos(lat_first_rad) * cos(lon_first_rad),  cos(lat_first_rad) * sin(lon_first_rad),  sin(lat_first_rad) }
        };

        std::vector<double> B= {
                ecef_point[0] - ref_point[0],
                ecef_point[1] - ref_point[1],
                ecef_point[2] - ref_point[2]
        };

        std::vector<double> C= {0, 0, 0};
        for( int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                C[i] += A[i][j]*B[j];
            }
        }

        return C;
    }

    void publish_odom_message(std::vector<double> enu_point,double orientation, ros::Time stamp){

        nav_msgs::Odometry odom_msg;

        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = "gps_odom";
        odom_msg.child_frame_id = "base_link";

        // Set position in NED frame
        odom_msg.pose.pose.position.x = enu_point[0];
        odom_msg.pose.pose.position.y = enu_point[1];
        odom_msg.pose.pose.position.z = 0;

        // Set orientation in quaternion format
        odom_msg.pose.pose.orientation = quaternionFromYaw(orientation);

        // Publish odometry message
        odom_pub.publish(odom_msg);

        ROS_INFO("odom trasformed message has been published X: %f, Y: %f, Z: %f", odom_msg.pose.pose.position.x,
                 odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom");

    Gps_to_odom odom;
    odom.init();

    return 0;
}