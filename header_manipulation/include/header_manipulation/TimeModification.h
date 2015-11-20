#ifndef TIME_MODIFICATION_H
#define TIME_MODIFICATION_H

#include <queue>
#include <boost/thread.hpp>
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
    TimeModification(const ros::NodeHandle &nh, ros::Rate &publish_rate);
    ~TimeModification(){}

    void processShapeShifter(const topic_tools::ShapeShifter::Ptr shape_shifter);
    void processShapeShifter(topic_tools::ShapeShifter &shape_shifter);
    void startPublisherThread(const ros::NodeHandle &nh);

private:
    typedef header_manipulation::TimeModificationConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    typedef std::pair<uint8_t[100000], ros::Time> StampedMsg;
    typedef std::queue<StampedMsg> StampedMsgBuffer;

    void configCB(Config &config, uint32_t level);
    void inputCB(const topic_tools::ShapeShifter::ConstPtr &input);
    void processBuffer(uint8_t *msg_buffer);
    void publishMsg(const StampedMsg &stamped_msg);
    void publishMsg(const uint8_t &msg_buffer, const ros::Time &time_to_pub);
    void publishMsgLoop(const ros::NodeHandle &nh);

    boost::thread msg_publisher_thread_;
    ros::NodeHandle private_nh_;
    ros::Subscriber generic_sub_;
    boost::shared_ptr<ReconfigureServer> reconf_server_;
    unsigned int seq_counter_;
    bool output_advertised_;
    ros::Publisher generic_pub_;
    boost::mutex buffer_mutex_, config_mutex_, pub_mutex_;
    StampedMsgBuffer stamped_msg_buffer_;

    double msg_delay_milliseconds_, time_offset_milliseconds_;
    ros::Duration msg_delay_, time_offset_;
    ros::Rate publish_retry_rate_;
};

#endif //TIME_MODIFICATION_H
