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

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomManipulation {
public:
  OdomManipulation();
  ~OdomManipulation();

private:
  void odomCB(const nav_msgs::OdometryConstPtr& odom);

  double angular_factor_, linear_factor_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
};

OdomManipulation::OdomManipulation() {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param("angular_factor", angular_factor_, 1.0);
  private_nh.param("linear_factor", linear_factor_, 1.0);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &OdomManipulation::odomCB, this);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("manipulated_odom", 1);
}

OdomManipulation::~OdomManipulation() {}

void OdomManipulation::odomCB (const nav_msgs::OdometryConstPtr& odom) {

  nav_msgs::Odometry manipulated_odom = *odom;

  manipulated_odom.twist.twist.angular.z *= angular_factor_;
  manipulated_odom.twist.twist.linear.x *= linear_factor_;

  odom_pub_.publish(manipulated_odom);
}

int main (int argc, char **argv) {

  ros::init(argc, argv, "odom_manipulation");

  OdomManipulation odom_manipulator;

  ros::spin();

  ros::shutdown();
  return 0;
}
