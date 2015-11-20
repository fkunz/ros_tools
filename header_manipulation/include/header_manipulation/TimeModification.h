#ifndef TIME_MODIFICATION_H
#define TIME_MODIFICATION_H

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <header_manipulation/TimeModificationConfig.h>
#include <laser_assembler/AssembleScans2.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <topic_tools/shape_shifter.h>
#include <tf/tf.h>

class TimeModification {

public:
    TimeModification(ros::Rate &rate);
    ~TimeModification(){}

private:
    typedef header_manipulation::TimeModificationConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

    ros::NodeHandle private_nh_;
    ros::Publisher generic_pub_;
    ros::Subscriber generic_sub_;
    boost::shared_ptr<ReconfigureServer> reconf_server_;

    bool output_advertised_;
    unsigned int seq_counter_;

    ros::Duration time_offset_;
    ros::Rate update_rate_;

    void configCB(Config &config, uint32_t level);
    void inputCB(const topic_tools::ShapeShifter::ConstPtr &input);
};

#endif //TIME_MODIFICATION_H
