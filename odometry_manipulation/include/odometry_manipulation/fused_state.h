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

#include <fstream>
#include <deque>
#include <iomanip>
#include <utility>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

class FusedState
{
public:
    struct MinimalTwist
    {
        double angular_z;
        double linear_x;
        double linear_y;
    };

    FusedState();
    ~FusedState();

    double getRoll();
    double getPitch();
    double getYaw();
    double getAngle();
    tf::Vector3 getAxis();
    MinimalTwist getCmdVel();
    MinimalTwist getCorrectedOdom();
    MinimalTwist getOdom();
    bool isStandingStill();
    bool isTurningInPlace();
    void updateFusedState(const bool turning_in_place, const bool standing_still);
    void updateFusedState(const nav_msgs::Odometry& odom, nav_msgs::Odometry& corrected_odom);
    void updateFusedState(const geometry_msgs::Quaternion& quat);
    void updateFusedState(const geometry_msgs::Twist& cmd_vel);

protected:

    double roll_;
    double pitch_;
    double yaw_;
    double angle_;
    tf::Vector3 axis_;
    MinimalTwist cmd_vel_;
    MinimalTwist corrected_odom_;
    MinimalTwist odom_;
    bool standing_still_;
    bool turning_in_place_;
};

FusedState::FusedState()
{

}

FusedState::~FusedState()
{

}

double FusedState::getRoll()
{
    return roll_;
}

double FusedState::getPitch()
{
    return pitch_;
}

double FusedState::getYaw()
{
    return yaw_;
}

double FusedState::getAngle()
{
    return angle_;
}

tf::Vector3 FusedState::getAxis()
{
    return axis_;
}

FusedState::MinimalTwist FusedState::getCmdVel()
{
    return cmd_vel_;
}

FusedState::MinimalTwist FusedState::getCorrectedOdom()
{
    return corrected_odom_;
}

FusedState::MinimalTwist FusedState::getOdom()
{
    return odom_;
}

bool FusedState::isStandingStill()
{
    return standing_still_;
}

bool FusedState::isTurningInPlace()
{
    return turning_in_place_;
}

void FusedState::updateFusedState(const bool turning_in_place, const bool standing_still)
{
    standing_still_ = standing_still;
    turning_in_place_ = turning_in_place;
}

void FusedState::updateFusedState(const nav_msgs::Odometry& odom, nav_msgs::Odometry& corrected_odom)
{
    odom_.angular_z = odom.twist.twist.angular.z;
    odom_.linear_x = odom.twist.twist.linear.x;
    odom_.linear_y = odom.twist.twist.linear.y;

    corrected_odom_.angular_z = corrected_odom.twist.twist.angular.z;
    corrected_odom_.linear_x = corrected_odom.twist.twist.linear.x;
    corrected_odom_.linear_y = corrected_odom.twist.twist.linear.y;
}

void FusedState::updateFusedState(const geometry_msgs::Quaternion& base_orientation_msg)
{
    tf::Quaternion base_orientation_quat;
    tf::quaternionMsgToTF(base_orientation_msg, base_orientation_quat);
    tf::Matrix3x3 base_orientation_mat(base_orientation_quat);
    base_orientation_mat.getEulerYPR(yaw_, pitch_, roll_);
    base_orientation_quat.setEuler(0.0, pitch_, roll_);
    angle_ = base_orientation_quat.getAngle();
    axis_ = base_orientation_quat.getAxis();
}

void FusedState::updateFusedState(const geometry_msgs::Twist& cmd_vel)
{
    cmd_vel_.angular_z = cmd_vel.angular.z;
    cmd_vel_.linear_x = cmd_vel.linear.x;
    cmd_vel_.linear_y = cmd_vel.linear.y;
}
