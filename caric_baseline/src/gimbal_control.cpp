#include "gimbal_control.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh_init;
    
    ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(nh_init);

    controller gb(nh_ptr);

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin();

    return 0;
}