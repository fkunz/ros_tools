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

#ifndef HEADER_MANIPULATION_H
#define HEADER_MANIPULATION_H

#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <header_manipulation/HeaderManipulationConfig.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <tf/tf.h>

class HeaderManipulation {
public:
    HeaderManipulation(ros::Rate &publish_rate);
    ~HeaderManipulation(){}

    void processShapeShifter(const topic_tools::ShapeShifter::Ptr shape_shifter);
    void processShapeShifter(topic_tools::ShapeShifter &shape_shifter);
    void startPublisherThread(const ros::NodeHandle &nh);

private:
    typedef header_manipulation::HeaderManipulationConfig Config;
    typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
    typedef std::pair<topic_tools::ShapeShifter, ros::Time> StampedMsg;
    typedef std::queue<boost::shared_ptr<StampedMsg> > StampedMsgBuffer;

    void configCB(Config &config, uint32_t level);
    void inputCB(const topic_tools::ShapeShifter::ConstPtr &input);
    void manipulateRawData(uint8_t *const msg_buffer);
    void publishMsg(const boost::shared_ptr<StampedMsg> stamped_msg);
    void publishMsg(const topic_tools::ShapeShifter &msg, const ros::Time &time_to_pub);
    void publishMsgLoop(const ros::NodeHandle &nh);

    ros::NodeHandle private_nh_;
    ros::Subscriber generic_sub_;
    boost::shared_ptr<ReconfigureServer> reconf_server_;
    bool output_advertised_;
    unsigned int seq_counter_;
    ros::Publisher generic_pub_;
    boost::mutex buffer_mutex_, config_mutex_, pub_mutex_;
    boost::thread msg_publisher_thread_;
    StampedMsgBuffer stamped_msg_buffer_;

    ros::Duration msg_delay_, time_offset_;
    ros::Rate publish_retry_rate_;
};

#endif  // HEADER_MANIPULATION_H
