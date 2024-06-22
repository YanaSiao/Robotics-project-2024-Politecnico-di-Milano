#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x;
    double y;
    double theta;
};

std::vector<Goal> readGoalsFromCSV(const std::string &filename) {
    std::vector<Goal> goals;
    std::ifstream file(filename);

    if (!file.is_open()) {
        ROS_ERROR("Could not open file %s", filename.c_str());
        return goals;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        Goal goal;

        std::getline(ss, token, ',');
        goal.x = std::stod(token);
        std::getline(ss, token, ',');
        goal.y = std::stod(token);
        std::getline(ss, token, ',');
        goal.theta = std::stod(token);

        goals.push_back(goal);
    }
    file.close();
    return goals;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    std::string csv_file;
    if (!nh.getParam("/goal_publisher/csv_file", csv_file)) {
        ROS_ERROR("Failed to get param 'csv_file'");
        return 1;
    }

    if (csv_file.empty()) {
        ROS_ERROR("CSV file parameter is empty");
        return 1;
    }

    ROS_INFO("CSV file: %s", csv_file.c_str());

    std::vector<Goal> goals = readGoalsFromCSV(csv_file);
    if (goals.empty()) {
        ROS_ERROR("No goals to send");
        return 1;
    }

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    for (const auto &goal : goals) {
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose.header.frame_id = "map";
        move_base_goal.target_pose.header.stamp = ros::Time::now();
        move_base_goal.target_pose.pose.position.x = goal.x;
        move_base_goal.target_pose.pose.position.y = goal.y;
        move_base_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.theta);

        ROS_INFO("Sending goal: x=%f, y=%f, theta=%f", goal.x, goal.y, goal.theta);
        ac.sendGoal(move_base_goal);

        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Reached goal");
        } else {
            ROS_WARN("Failed to reach goal, trying next one");
        }

        ros::Duration(5).sleep();  // Delay before sending next goal
    }

    return 0;
}
