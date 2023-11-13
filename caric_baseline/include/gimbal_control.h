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

class controller
{
    public:
    controller(ros::NodeHandlePtr &nh_ptr_)
    : nh_ptr(nh_ptr_)
    {   
        
        //target_point_sub = nh_ptr->subscribe()
        // odom_sub_ = nh_ptr->subscribe("/ground_truth/odometry", 10, &controller::OdomCallback, this);
        // gimbal_sub_ = nh_ptr->subscribe("/firefly/gimbal", 10, &controller::GimbalCallback, this);
        gimbal_pub_ = nh_ptr_->advertise<geometry_msgs::Twist>("/firefly/command/gimbal", 1);
        gimbal_msg = gimbal_msg_build();
        gimbal_pub_.publish(gimbal_msg);


    }
    private:
    geometry_msgs::Twist gimbal_msg;
    ros::Subscriber odom_sub_;   // Get neibor_info update
    ros::Subscriber gimbal_sub_; // Get gimbal info update;
    ros::Publisher gimbal_pub_; 

    /*void OdomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        if(!map_initialise)
        {
            Eigen::Vector3d initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            std_msgs::String init_position_msg;
            init_position_msg.data="init_pos;"+nh_ptr->getNamespace()+";"+to_string(initial_position.x())+","+to_string(initial_position.y())+","+to_string(initial_position.z());
            if(communication_initialise)
            {
                communication_pub_.publish(init_position_msg);
            }
            return;
        }

        Eigen::Vector3d my_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Eigen::Matrix3d R = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();
        mm.update_position(my_position, R);
    }
     void GimbalCallback(const geometry_msgs::TwistStamped &msg)
    {
        Eigen::Vector3d position = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
        Eigen::Matrix3d gimbal_rotation_matrix = Rpy2Rot(gimbal_position);
        Eigen::Matrix3d now_rot = gimbal_rotation_matrix * drone_rotation_matrix;
        Eigen::Vector3d rpy = Rot2rpy(now_rot);
        rpy.x() = 0;
    }*/
    geometry_msgs::Twist gimbal_msg_build()
    {
        ros::NodeHandlePtr nh_ptr;
        geometry_msgs::Twist gimbal_msg;
        gimbal_msg.linear.x = -1; // setting linear.x to -1.0 enables velocity control mode.
        // if (fabs(target_euler_rpy.z()) < M_PI / 2)
        // {
        gimbal_msg.linear.y = 0; // if linear.x set to 1.0, linear,y and linear.z are the
        gimbal_msg.linear.z = 0; // target pitch and yaw angle, respectively.
        // }
        // else
        // {
        //     gimbal_msg.linear.y = target_euler_rpy.y(); // if linear.x set to 1.0, linear,y and linear.z are the
        //     gimbal_msg.linear.z = 0;                    // target pitch and yaw angle, respectively.
        // }   
        
        gimbal_msg.angular.x = 0.0;
        gimbal_msg.angular.y = 0.2; // in velocity control mode, this is the target pitch velocity
        gimbal_msg.angular.z = 0.2; // in velocity control mode, this is the target yaw velocity
        return gimbal_msg;
    }
};

