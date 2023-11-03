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





class Agent
{
public:
    Agent(ros::NodeHandlePtr &nh_ptr_)
    : nh_ptr(nh_ptr_)
    {
        //把该explorer需要的下一个位置通过/task_assign传进来，在回调中直接用cmd_pub_0、cmd_pub_1发送控制指令
        task_sub_ = nh_ptr->subscribe("/task_assign" + nh_ptr->getNamespace(), 10, &Agent::TaskCallback, this);
        
        //-------------------------------------------------------------------------------------------------
        //处理/broadcast的LOS问题
        client    = nh_ptr->serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");
        communication_pub_ = nh_ptr->advertise<std_msgs::String>("/broadcast", 10);

        string str = nh_ptr->getNamespace();
        str.erase(0, 1);
        srv.request.source = str;
        srv.request.targets.push_back("all");
        srv.request.topic_name = "/broadcast";
        srv.request.package_name = "std_msgs";
        srv.request.message_type = "String";
        while (!serviceAvailable)
        {
            serviceAvailable = ros::service::waitForService("/create_ppcom_topic", ros::Duration(10.0));
        }
        string result = "Begin";
        while (result != "success lah!")
        {
            client.call(srv);
            result = srv.response.result;
            printf(KYEL "%s\n" RESET, result.c_str());
            std::this_thread::sleep_for(chrono::milliseconds(1000));//c++进程休眠1秒
        }
        communication_initialise = true;
        //实现LOS，通过订阅/broadcast/Namespace代替订阅/broadcast
        //-------------------------------------------------------------------------------------------------

        //拿到位置后用communication_pub_传到gcs里用以初始定位并分配box顺序。
        odom_sub_        = nh_ptr->subscribe("/ground_truth/odometry", 10, &Agent::OdomCallback, this);//map

        cmd_pub = nh_ptr ->advertise<trajectory_msgs::MultiDOFJointTrajectory>(nh_ptr->getNamespace() + "/command/trajectory",1);
    }

private:
    ros::NodeHandlePtr nh_ptr;  // nodehandle for communication\\

    ros::Publisher cmd_pub;//发送控制指令

    /*ros::Timer TimerProbeNbr;   // To request updates from neighbours
    ros::Timer TimerPlan;       // To design a trajectory
    ros::Timer TimerCmdOut;     // To issue control setpoint to unicon
    ros::Timer TimerViz;        // To vizualize internal states
    */


    // part 1
    caric_mission::CreatePPComTopic srv; // This PPcom create for communication between neibors;
    ros::ServiceClient client;           // The client to create ppcom
    ros::Publisher communication_pub_;   // PPcom publish com
    bool serviceAvailable = false;       // The flag whether the communication service is ready
    ros::Subscriber task_sub_;
    ros::Subscriber odom_sub_;
    //ros::Subscriber com_sub_;
    //string pre_task;

    /*
    // callback Q2
    ros::Subscriber odom_sub_;   // Get neibor_info update
    ros::Subscriber gimbal_sub_; // Get gimbal info update;
    // callback Q3
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *nbr_sub_;
    message_filters::Subscriber<nav_msgs::Odometry>       *odom_filter_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2,
                                                            nav_msgs::Odometry> MySyncPolicy;
    // // boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    message_filters::Synchronizer<MySyncPolicy> *sync_;

    // callback Q4
    ros::Publisher motion_pub_; // motion command pub
    ros::Publisher gimbal_pub_; // motion gimbal pub

    // callback Q5
    ros::Publisher map_marker_pub_;
    ros::Publisher path_pub_;

    mainbrain mm;

    // variable for static map
    vector<Eigen::Vector3d> Nbr_point;
    */

    bool communication_initialise = false;
    // Callback function

    //输入需要xyz、偏航角
    void TaskCallback(const std_msgs::String msg)
    {
        Eigen::Vector3d target_pos(0,0,0);

        vector<string> spilited_str;
        std::istringstream iss(msg.data);
        std::string substring;
        while (std::getline(iss, substring, ';'))
        {
            spilited_str.push_back(substring);
        }

        if(nh_ptr->getNamespace() == "/jurong")
        {
            target_pos=str2point(spilited_str[0]);
        }
        if(nh_ptr->getNamespace() == "/raffles")
        {
            target_pos=str2point(spilited_str[1]);
        }
        
        double target_yaw=0;

        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;

        geometry_msgs::Transform transform_msg;//Vector3 translation、Quaternion rotation
        geometry_msgs::Twist accel_msg, vel_msg;

        transform_msg.translation.x = target_pos(0);
        transform_msg.translation.y = target_pos(1);
        transform_msg.translation.z = target_pos(2);
        transform_msg.rotation.x = 0;
        transform_msg.rotation.y = 0;
        transform_msg.rotation.z = sinf(target_yaw*0.5);
        transform_msg.rotation.w = cosf(target_yaw*0.5);

        trajpt_msg.transforms.push_back(transform_msg);

        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        accel_msg.linear.x = 0;
        accel_msg.linear.x = 0;
        accel_msg.linear.x = 0;

        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajset_msg.header.stamp = ros::Time::now();
        cmd_pub.publish(trajset_msg);
    }

    void OdomCallback(const nav_msgs::OdometryConstPtr &msg) //把里程计的数据pub到broadcast（与gcs的positionCallback相关）
    {
        //if(!)
        //{
        Eigen::Vector3d initial_position = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        std_msgs::String init_position_msg;
        init_position_msg.data="init_pos;"+nh_ptr->getNamespace()+";"+to_string(initial_position.x())+","+to_string(initial_position.y())+","+to_string(initial_position.z());
        if(communication_initialise)
        {
            communication_pub_.publish(init_position_msg);
        }
        return;
        //}
    }

    Eigen::Vector3d str2point(string input)
        {
            Eigen::Vector3d result;
            std::vector<string> value;
            boost::split(value, input, boost::is_any_of(","));//x,y,z
            // cout<<input<<endl;
            if (value.size() == 3)
            {
                result = Eigen::Vector3d(stod(value[0]), stod(value[1]), stod(value[2]));
            }
            else
            {
                cout << input << endl;
                cout << "error use str2point 2" << endl;
            }
            return result;
        }
};