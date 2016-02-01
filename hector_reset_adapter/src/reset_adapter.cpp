#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

//namespace hector_reset_adapter {

//class ResetAdapter
//{
//public:
//    ResetAdapter();

//private:
//    void syscommandCB(const std_msgs::StringConstPtr &syscommand_ptr);

//    ros::ServiceClient mapper_reset_client;
//    std_srvs::Empty reset_srv;
//};

//ResetAdapter::ResetAdapter()
//{
//    ros::NodeHandle nh;
//    ros::Subscriber syscommand_sub_ = nh.subscribe("syscommand", 10, &ResetAdapter::syscommandCB, this);
//    mapper_reset_client = nh.serviceClient<std_srvs::Empty>("mapper/reset");
//}


//void ResetAdapter::syscommandCB(const std_msgs::StringConstPtr &syscommand_ptr)
//{
//    if (syscommand_ptr->data == "reset")
//    {
//        mapper_reset_client.call(reset_srv);
//    } else
//    {
//        ROS_DEBUG("Ignore syscommand. %s", syscommand_ptr->data.c_str());
//    }
//}
//};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reset_adapter");

//    hector_reset_adapter::ResetAdapter reset_adapter;

    ros::spin();

    return 0;
}
