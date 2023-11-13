// Just for case
#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "Eigen/Dense"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Astar.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/geometry/distance.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"
#include <mutex>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <caric_mission/CreatePPComTopic.h>
#include <std_msgs/String.h>

#include "general_task_init.h"
#include "Astar.h"
#include <map>

// # if xxx still sending msgs:
// #     x, y, z = xxxx[newest] (subscript)
// #         if waypoint is completed (all x, y,z are valid):
// #             waypoint.append(x, y, z)
// #         else:
// #             [some modifications]
//
// # for waypoints in [waypoint]:
// #     if current position - waypoints < 10:
// #         photographer1 fly to this waypoint by fetching it this waypoints
// #         photographer2 fly to this waypoint by fetching it this waypoints + x

// Program started
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <tuple>
#include <cmath>
#include <vector>

// Global vector that is used to store waypoints
std::vector<std::tuple<double, double, double>> waypoint;

// Global variables for self status
double p1x, p1y, p1z;

// Callback function to process waypoint messages
void waypointCallback(const std_msgs::String::ConstPtr &msg)
{
    std::istringstream msg_stream(msg->data);

    std::string token;
    std::string pointLabel;
    double x, y, z;
    double normal_x, normal_y, normal_z;

    // Iterate over tokens in the message
    while (getline(msg_stream, token, ','))
    {
        std::istringstream token_stream(token);
        token_stream.ignore(256, '('); // Ignore up to the opening parenthesis

        if (pointLabel == "Point")
        {
            token_stream >> x >> std::ws >> token >> std::ws >> y >> std::ws >> token >> std::ws >> z;
        }
        else if (pointLabel == "Normal")
        {
            token_stream >> normal_x >> std::ws >> token >> std::ws >> normal_y >> std::ws >> token >> std::ws >> normal_z;
        }

        // Move to the next label (e.g., BoundingBoxId, Normal) in the next iteration
        getline(msg_stream, pointLabel, ':');
    }

    // Modify x, y, z based on the normal vector adjustment (e.g., adding 10 times the normal vector)
    x += 10 * normal_x;
    y += 10 * normal_y;
    z += 10 * normal_z;

    waypoint.emplace_back(x, y, z);
}

// Callback function to update self status
void selfStatusCallback(const nav_msgs::OdometryConstPtr &msg)
{
    p1x = msg->pose.pose.position.x;
    p1y = msg->pose.pose.position.y;
    p1z = msg->pose.pose.position.z;
}

bool still_sending_msgs(const ros::Time &start_time, const ros::Duration &timeout)
{
    // Check if the channel is still sending messages within the specified timeout
    return (ros::Time::now() - start_time) < timeout && ros::ok();
}

bool is_waypoint_completed(double x, double y, double z)
{
    // Check if the waypoint (x, y, z) is not empty
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double calculate_distance(std::tuple<double, double, double> uav_pos, std::tuple<double, double, double> waypoint_pos)
{
    double uavx, uavy, uavz, waypointx, waypointy, waypointz;
    std::tie(uavx, uavy, uavz) = uav_pos;
    std::tie(waypointx, waypointy, waypointz) = waypoint_pos;

    return std::sqrt(std::pow(uavx - waypointx, 2) + std::pow(uavy - waypointy, 2) + std::pow(uavz - waypointz, 2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "photographer");
    ros::NodeHandle nh;

    // Define and initialize <unit_id>
    std::string unit_id;
    // Check if the parameter has been set, if not, provide a default value
    if (!nh.getParam("/unit_id", unit_id))
    {
        // Handle the case where the parameter is not set
        ROS_WARN("Param 'unit_id' not set!!!!!!!!!!!!!!!!!!!");
        return 1;
    }

    std::string self_status_topic = "/" + unit_id + "/ground_truth/odometry"; // Define the unit_id in launch file
    std::string uav_fly_topic = "/" + unit_id + "/command/trajectory";
    std::string explorer_waypoint_topic = "/task_assign/" + unit_id;

    // Message subscribers (waypoints + self_status)
    // listens to the 'explorer_waypoint_topic' topic, use a message queue of size 10, and call the waypointCallback function whenever a new message arrives
    ros::Subscriber waypoint_sub = nh.subscribe("explorer_waypoint_topic", 10, waypointCallback);
    ros::Subscriber self_status_sub = nh.subscribe(self_status_topic, 10, selfStatusCallback);

    // Message publisher (fly_command)
    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(uav_fly_topic, 10);

    // Set a timeout of 10 seconds (adjust as needed)
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(10.0);

    while (ros::ok() && still_sending_msgs(start_time, timeout))
    {
        ros::spinOnce(); // Handle callbacks and publish messages
    }

    // Loop all the x, y, z in the waypoint list [x1, y1, z1, x2, y2, z2, ... xk, yk, zk]
    for (size_t i = 0; i < waypoint.size(); i += 3)
    {
        double x = std::get<0>(waypoint[i]);
        double y = std::get<1>(waypoint[i + 1]);
        double z = std::get<2>(waypoint[i + 2]);

        // Update the current status of the photographer in every loop
        ros::spinOnce(); // Ensure that self status is updated

        // Replace {threshold} with the actual threshold value
        double threshold = 30;
        if (calculate_distance(std::make_tuple(p1x, p1y, p1z), std::make_tuple(x, y, z)) < threshold)
        {
            // Create trajectory message and populate it with target position
            trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

            geometry_msgs::Transform transform_msg;
            geometry_msgs::Twist accel_msg, vel_msg;

            transform_msg.translation.x = x;
            transform_msg.translation.y = y;
            transform_msg.translation.z = z;
            transform_msg.rotation.x = 0;
            transform_msg.rotation.y = 0;
            transform_msg.rotation.z = 0;
            transform_msg.rotation.w = 1;

            trajpt_msg.transforms.push_back(transform_msg);

            // Set desired velocity and acceleration to 0 (hover)
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            accel_msg.linear.x = 0;
            accel_msg.linear.y = 0;
            accel_msg.linear.z = 0;

            trajpt_msg.velocities.push_back(vel_msg);
            trajpt_msg.accelerations.push_back(accel_msg);
            trajset_msg.points.push_back(trajpt_msg);

            trajset_msg.header.stamp = ros::Time::now();
            trajectory_pub.publish(trajset_msg);

            // Simulate flying to the waypoint by removing it from the list
            waypoint.erase(waypoint.begin() + i, waypoint.begin() + i + 3);
        }
    }
    return 0;
}