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

#include <header_manipulation/PointcloudManipulation.h>

ScanManipulation::ScanManipulation(ros::Rate &publish_retry_rate) :
    publish_retry_rate_(publish_retry_rate)
{
    nh_ = ros::NodeHandle("");
    laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("laser_in", 100,
                                                                    &ScanManipulation::laserCB, this);
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 10,
                                                                    &ScanManipulation::pointcloudCB, this);

    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>("laser_out", 100);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 10);

    ros::NodeHandle private_nh("~");
    reconf_server_.reset(new ReconfigureServer(private_nh));
    reconf_server_->setCallback(boost::bind(&ScanManipulation::configCB, this, _1, _2));

    private_nh.param("frame_id_new", frame_id_new_, frame_id_new_);
    private_nh.param("frame_id_to_replace", frame_id_to_replace_, frame_id_to_replace_);
    double msg_delay_in_milliseconds, time_offset_in_milliseconds;
    private_nh.param("msg_delay_milliseconds", msg_delay_in_milliseconds, msg_delay_.toSec() * 1000);
    private_nh.param("time_offset_milliseconds", time_offset_in_milliseconds, time_offset_.toSec() * 1000);
    msg_delay_ = ros::Duration(msg_delay_in_milliseconds/1000);
    time_offset_ = ros::Duration(time_offset_in_milliseconds/1000);
    if (private_nh.hasParam("publish_retry_rate"))
    {
        double retry_rate;
        private_nh.param("publish_retry_rate", retry_rate, 100.0);
        publish_retry_rate_ = ros::Rate(retry_rate);
    }
}

void ScanManipulation::configCB(Config &config, uint32_t level)
{
    ROS_INFO("PointcloudManipulation configCB");
    frame_id_new_ = config.frame_id_new;
    frame_id_to_replace_ = config.frame_id_to_replace;
    msg_delay_ = ros::Duration(config.msg_delay_milliseconds/1000);
    publish_retry_rate_ = ros::Rate(config.publish_retry_rate);
    time_offset_ = ros::Duration(config.time_offset_milliseconds/1000);
}

void ScanManipulation::laserCB(const sensor_msgs::LaserScan::ConstPtr &input)
{
    ros::Time start_time = ros::Time::now();
    ROS_DEBUG("PointcloudManipulation laserCB. seq: %i", input->header.seq);
    sensor_msgs::LaserScan::Ptr output(new sensor_msgs::LaserScan(*input));
    output->header.stamp += time_offset_;
    if (output->header.frame_id == frame_id_to_replace_)
        output->header.frame_id = frame_id_new_;
    ros::Time time_to_publish = start_time + msg_delay_;
    while (ros::Time::now() < time_to_publish)
        publish_retry_rate_.sleep();
    laser_pub_.publish(output);
}

void ScanManipulation::pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    ROS_DEBUG("PointcloudManipulation pointcloudCB");
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2(*input));
    output->header.stamp += time_offset_;
    if (output->header.frame_id == frame_id_to_replace_)
        output->header.frame_id = frame_id_new_;
    ros::Time time_to_publish = output->header.stamp + msg_delay_;
    while (ros::Time::now() < time_to_publish)
        publish_retry_rate_.sleep();
    pointcloud_pub_.publish(output);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_Manipulation_node");
    ros::NodeHandle nh("");
    ros::Rate publish_retry_rate(10);

    ScanManipulation scan_Manipulation_node(publish_retry_rate);
    ros::spin();

    return 0;
}
