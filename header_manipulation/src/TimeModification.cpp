#include <header_manipulation/TimeModification.h>

TimeModification::TimeModification(ros::Rate &publish_rate) : publish_rate_(publish_rate) {
    private_nh_ = ros::NodeHandle("~");
    generic_sub_ = private_nh_.subscribe<topic_tools::ShapeShifter>("input", 10, &TimeModification::inputCB, this);

    reconf_server_.reset(new ReconfigureServer(private_nh_));
    reconf_server_->setCallback(boost::bind(&TimeModification::configCB, this, _1, _2));

    output_advertised_ = false;
    seq_counter_ = 0;
}

void TimeModification::configCB(Config &config, uint32_t level) {
    msg_delay_ = ros::Duration(config.msg_delay_milliseconds/1000);
    publish_rate_ = ros::Rate(config.publish_rate);
    time_offset_ = ros::Duration(config.time_offset_milliseconds/1000);
}

void TimeModification::inputCB(const topic_tools::ShapeShifter::ConstPtr &input) {
    ROS_DEBUG("in callback");
    ros::Time start_time(ros::Time::now());
    if (!output_advertised_) {
        ROS_INFO("Output not advertised. Setting up publisher now.");
        ros::AdvertiseOptions opts("output", 1, input->getMD5Sum(),
                                   input->getDataType(), input->getMessageDefinition());
        generic_pub_ = private_nh_.advertise(opts);
        output_advertised_ = true;
    }
    uint8_t msg_buffer[input->size()];
    ros::serialization::OStream o_stream(msg_buffer, input->size());
    ros::serialization::IStream i_stream(msg_buffer, input->size());
    input->write(o_stream);
    manipulateRawData(msg_buffer);
    topic_tools::ShapeShifter output;
    output.read(i_stream);
    publishMsg(output, start_time + msg_delay_);
}

void TimeModification::manipulateRawData(uint8_t *const msg_buffer)
{
    std_msgs::Header header;
    header.seq = ((uint32_t *)msg_buffer)[0];
    header.stamp.sec = ((uint32_t *)msg_buffer)[1];
    header.stamp.nsec = ((uint32_t *)msg_buffer)[2];

    // TODO (fkunz) check if header data is consistent
    if ( fabs((header.stamp - ros::Time::now()).toSec()) > 5.0 ) {
        ROS_WARN("Probably subscribed to a topic with a message type not "
                 "containing a header.Timestamp is: %f at Time: %f",
                 header.stamp.toSec(), ros::Time::now().toSec());
    }
    {
        header.stamp += time_offset_;
    }
    ((uint32_t *)msg_buffer)[0] = header.seq + seq_counter_++;
    ((uint32_t *)msg_buffer)[1] = header.stamp.sec;
    ((uint32_t *)msg_buffer)[2] = header.stamp.nsec;
}

void TimeModification::publishMsg(const topic_tools::ShapeShifter &msg, const ros::Time time_to_pub)
{
    ros::Time end_time(ros::Time::now());
    ros::Time last_time;
    do
    {
        last_time = end_time;
        ROS_DEBUG("waiting to publish msg at %f", time_to_pub.toSec());
        if (end_time > time_to_pub)
        {
            ROS_DEBUG("publishing msg which should be held back till: %f", time_to_pub.toSec());
            generic_pub_.publish(msg);
            return;
        }
        publish_rate_.sleep();
        end_time = ros::Time::now();
    } while (last_time <= end_time);
    ROS_WARN("Detected jump back in time. Dropping msg. last: %f, end %f",
             last_time.toSec(), end_time.toSec());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "time_modification_node");
    ros::start();
    ros::Rate publish_rate(60);

    TimeModification time_modification_node(publish_rate);
    ros::spin();

    return 0;
}
