//=================================================================================================
// Copyright (c) 2015, Florian Kunz, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <header_manipulation/HeaderManipulation.h>

TimeModification::TimeModification(ros::Rate &publish_rate) :
    publish_retry_rate_(publish_rate)
{
    private_nh_ = ros::NodeHandle("~");
    generic_sub_ = private_nh_.subscribe<topic_tools::ShapeShifter>("input", 10,
                                                                    &TimeModification::inputCB, this);

    reconf_server_.reset(new ReconfigureServer(private_nh_));
    reconf_server_->setCallback(boost::bind(&TimeModification::configCB, this, _1, _2));

    output_advertised_ = false;
    seq_counter_ = 0;
}

void TimeModification::processShapeShifter(const topic_tools::ShapeShifter::Ptr shape_shifter)
{
    ROS_DEBUG("TimeModification processShapeShifter Ptr");
    processShapeShifter(*shape_shifter);
}

void TimeModification::processShapeShifter(topic_tools::ShapeShifter &shape_shifter)
{
    ROS_DEBUG("TimeModification processShapeShifter");
    uint8_t msg_buffer[shape_shifter.size()];
    ros::serialization::OStream o_stream(msg_buffer, shape_shifter.size());
    manipulateRawData(msg_buffer);
    ros::serialization::IStream i_stream(msg_buffer, shape_shifter.size());
    shape_shifter.read(i_stream);
}

void TimeModification::startPublisherThread(const ros::NodeHandle &nh)
{
    ROS_DEBUG("TimeModification startPublisherThread");
    boost::thread pub_thread(&TimeModification::publishMsgLoop, this, nh);
    pub_thread.detach();
}

void TimeModification::configCB(Config &config, uint32_t level)
{
    ROS_INFO("TimeModification configCB");
    boost::mutex::scoped_lock config_lock(config_mutex_);
    msg_delay_ = ros::Duration(config.msg_delay_milliseconds/1000);
    publish_retry_rate_ = ros::Rate(config.publish_retry_rate);
    time_offset_ = ros::Duration(config.time_offset_milliseconds/1000);
}

void TimeModification::inputCB(const topic_tools::ShapeShifter::ConstPtr &input)
{
    ROS_DEBUG("TimeModification inputCB");
    ros::Time start_time(ros::Time::now());
    // Initialize.
    if (!output_advertised_) {
        ROS_INFO("Output not advertised. Setting up publisher now.");
        ros::AdvertiseOptions opts("output", 1, input->getMD5Sum(),
                                   input->getDataType(), input->getMessageDefinition());
        boost::mutex::scoped_lock pub_lock(pub_mutex_);
        generic_pub_ = private_nh_.advertise(opts);
        output_advertised_ = true;
    }
    // Copy shape shifter data to array.
    uint8_t msg_buffer[input->size()];
    ros::serialization::OStream o_stream(msg_buffer, input->size());
    input->write(o_stream);
    // Manipulate (header) data.
    manipulateRawData(msg_buffer);
    // Read data from array to StampedMsg
    boost::shared_ptr<StampedMsg> stamped_msg(new StampedMsg());
    stamped_msg->second = start_time + msg_delay_;
    ros::serialization::IStream i_stream(msg_buffer, input->size());
    stamped_msg->first.read(i_stream);
    // Push StampedMsg to Buffer for multithreaded publishing.
    boost::mutex::scoped_lock buffer_lock(buffer_mutex_);
    stamped_msg_buffer_.push(stamped_msg);
    ROS_DEBUG("all done");
}

void TimeModification::manipulateRawData(uint8_t *const msg_buffer)
{
    ROS_DEBUG("TimeModification manipulateRawData");
    std_msgs::Header header;;
    header.seq = ((uint32_t *)msg_buffer)[0];
    header.stamp.sec = ((uint32_t *)msg_buffer)[1];
    header.stamp.nsec = ((uint32_t *)msg_buffer)[2];

    // TODO (fkunz) check if header data is consistent
    if ( fabs((header.stamp - ros::Time::now()).toSec()) > 5.0 )
    {
        ROS_WARN("Probably subscribed to a topic with a message type not "
                 "containing a header.Timestamp is: %f at Time: %f",
                 header.stamp.toSec(), ros::Time::now().toSec());
    }
    boost::mutex::scoped_lock config_lock(config_mutex_);
    ROS_DEBUG("setting header stamp from %f to %f", header.stamp.toSec(),
              (header.stamp + time_offset_).toSec());
    header.stamp += time_offset_;

    ((uint32_t *)msg_buffer)[0] = header.seq + seq_counter_++;
    ((uint32_t *)msg_buffer)[1] = header.stamp.sec;
    ((uint32_t *)msg_buffer)[2] = header.stamp.nsec;
}

void TimeModification::publishMsgLoop(const ros::NodeHandle &nh)
{
    ROS_DEBUG("TimeModification publishMsgLoop");
    while (nh.ok())
    {
        {
            boost::mutex::scoped_lock buffer_lock(buffer_mutex_);
            if (!stamped_msg_buffer_.empty())
            {
                ROS_DEBUG("publishing msg");
                publishMsg(stamped_msg_buffer_.front());
                stamped_msg_buffer_.pop();
                continue;
            }
        }
        publish_retry_rate_.sleep();
    }
}

void TimeModification::publishMsg(const boost::shared_ptr<StampedMsg> stamped_msg)
{
    ROS_DEBUG("TimeModification publishMsg");
    publishMsg(stamped_msg->first, stamped_msg->second);
}

void TimeModification::publishMsg(const topic_tools::ShapeShifter &msg, const ros::Time &time_to_pub)
{
    ROS_DEBUG("TimeModification publishMsg Thread started with timestamp %f", time_to_pub.toSec());
    ros::Time end_time(ros::Time::now());
    ros::Time last_time;
    config_mutex_.lock();
    const ros::Duration initial_delay(msg_delay_);
    config_mutex_.unlock();
    do
    {
        last_time = end_time;
        ROS_DEBUG("waiting to publish msg at %f", time_to_pub.toSec());
        if (end_time > time_to_pub)
        {
            boost::mutex::scoped_lock pub_lock(pub_mutex_);
            ROS_DEBUG("publishing msg which should be held back till: %f", time_to_pub.toSec());
            generic_pub_.publish(msg);
            return;
        }
        boost::mutex::scoped_lock config_lock(config_mutex_);
        publish_retry_rate_.sleep();
        end_time = ros::Time::now();
        if (initial_delay != msg_delay_)
        {
            ROS_WARN("Detected change of msg_delay. Dropping msg to be published at %f",
                     time_to_pub.toSec());
            return;
        }
    } while (last_time <= end_time);
    ROS_WARN("Detected jump back in time. Dropping msg. last: %f, end %f",
             last_time.toSec(), end_time.toSec());
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "header_manipulation_node");
    ros::NodeHandle nh;
    ros::Rate publish_rate(10);

    TimeModification header_manipulation_node(publish_rate);
    ROS_INFO("Constructor successfull start spinning.");
    header_manipulation_node.startPublisherThread(nh);
    ROS_INFO("Publisher Thread started.");
    ros::spin();

    return 0;
}
