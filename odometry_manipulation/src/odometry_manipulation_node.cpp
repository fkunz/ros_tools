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
#include <odometry_manipulation/fused_state.h>

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

class OdomManipulation
{
public:
    OdomManipulation(ros::Rate* const rate);
    ~OdomManipulation();

private:
    void checkFusedStateForSlip(FusedState& fused_state);
    void storeFusedState(const FusedState& fused_state);
    void cmdVelCB(const geometry_msgs::TwistConstPtr& cmd_vel);
    void imuCB(const sensor_msgs::ImuConstPtr& imu);
    void odomCB(const nav_msgs::OdometryConstPtr& odom);

    double angular_factor_, linear_factor_;
    double angle_deviation_threshold_, angular_speed_threshold_, linear_speed_threshold_;
    unsigned int lookback_limit_, min_steps_, state_buffer_limit_;
    ros::Rate* const update_rate_;

    ros::Subscriber cmd_vel_sub_, imu_sub_, odom_sub_;
    ros::Publisher odom_pub_;

    bool standing_still_, turning_in_place_;
    geometry_msgs::Twist last_cmd_vel_;
    FusedState current_fused_state_;
    std::deque<FusedState> state_buffer_;

    std::ofstream logfile;
};

OdomManipulation::OdomManipulation(ros::Rate* const rate): update_rate_(rate)
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param("angular_factor", angular_factor_, 1.0);
    private_nh.param("linear_factor", linear_factor_, 1.0);
    angle_deviation_threshold_ = 0.002;
    angular_speed_threshold_ = 0.005;
    linear_speed_threshold_ = 0.005;
    lookback_limit_ = 100;
    min_steps_ = 10;
    state_buffer_limit_ = 1000;

    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel_raw", 10, &OdomManipulation::cmdVelCB, this);
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>("imu_quat", 10, &OdomManipulation::imuCB, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &OdomManipulation::odomCB, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("manipulated_odom", 1);

    current_fused_state_ = FusedState();
    standing_still_ = true;
    turning_in_place_ = true;

    logfile.open("odom_manipulator.csv");
    logfile << "Time, roll, pitch, yaw, total angle, axis.x, axis.y, axis.z, turning in place, cmd_vel.z, cmd_vel.x, corrected_odom.z, corrected_odom.x, odom.z, odom.x";
    logfile << std::fixed << std::setw( 24 ) << std::setprecision( 8 ) << std::setfill( ' ' );
    logfile << "\n";
}

OdomManipulation::~OdomManipulation()
{
    logfile.close();
}

void OdomManipulation::checkFusedStateForSlip(FusedState& fused_state)
{
    logfile << ros::Time::now().toSec();
    logfile << ", " << fused_state.getRoll();
    logfile << ", " << fused_state.getPitch();
    logfile << ", " << fused_state.getYaw();
    logfile << ", " << fused_state.getAngle();
    logfile << ", " << fused_state.getAxis().x();
    logfile << ", " << fused_state.getAxis().y();
    logfile << ", " << fused_state.getAxis().z();
    logfile << ", " << 0.02 * (5 + fused_state.isTurningInPlace() + fused_state.isStandingStill());
    logfile << ", " << fused_state.getCmdVel().angular_z;
    logfile << ", " << fused_state.getCmdVel().linear_x;
    logfile << ", " << fused_state.getCorrectedOdom().angular_z;
    logfile << ", " << fused_state.getCorrectedOdom().linear_x;
    logfile << ", " << fused_state.getOdom().angular_z;
    logfile << ", " << fused_state.getOdom().linear_x;
    if (turning_in_place_)
    {
        bool slip_detected = false;
        unsigned int steps = 0;
        double average_angle = 0;
        for (size_t i = 0; i < lookback_limit_ && i < state_buffer_.size(); ++i)
        {
            if (!state_buffer_[i].isTurningInPlace())
                break;
            ++steps;
            average_angle += state_buffer_[i].getAngle();
        }
        if (steps > min_steps_)
        {
            average_angle /= steps;
            double mean_error = 0.0;
            for (size_t j = 0; j < steps; ++j)
            {
                mean_error += fabs(state_buffer_[j].getAngle() - average_angle);
            }
            mean_error /= steps;
            if (mean_error > angle_deviation_threshold_)
                slip_detected = true;
            else
            {
                ROS_WARN("roll: % 2.4f pitch: % 2.4f total angle: % 2.4f\n total axis: % 2.4f, % 2.4f, % 2.4f           steps: %i, mean: % 2.4f",
                         fused_state.getRoll(), fused_state.getPitch(), fused_state.getAngle(),
                         fused_state.getAxis().getX(), fused_state.getAxis().getY(), fused_state.getAxis().getZ(),
                         steps, mean_error);
            }
        }

        if (slip_detected)
            ROS_ERROR("roll: % 2.4f pitch: % 2.4f total angle: % 2.4f\n total axis: % 2.4f, % 2.4f, % 2.4f",
                      fused_state.getRoll(), fused_state.getPitch(), fused_state.getAngle(),
                      fused_state.getAxis().getX(), fused_state.getAxis().getY(), fused_state.getAxis().getZ());
        else
            ROS_INFO("roll: % 2.4f pitch: % 2.4f total angle: % 2.4f\n total axis: % 2.4f, % 2.4f, % 2.4f",
                     fused_state.getRoll(), fused_state.getPitch(), fused_state.getAngle(),
                     fused_state.getAxis().getX(), fused_state.getAxis().getY(), fused_state.getAxis().getZ());
        logfile << ", " << 0.08 + 0.02*slip_detected;
    }
    else
        ROS_DEBUG("roll: % 2.4f pitch: % 2.4f total angle: % 2.4f\n total axis: % 2.4f, % 2.4f, % 2.4f",
                  fused_state.getRoll(), fused_state.getPitch(), fused_state.getAngle(),
                  fused_state.getAxis().getX(), fused_state.getAxis().getY(), fused_state.getAxis().getZ());

    logfile << "\n";
}

void OdomManipulation::storeFusedState(const FusedState& fused_state)
{
    if (state_buffer_.size() >= state_buffer_limit_)
        state_buffer_.pop_back();
    state_buffer_.push_front(fused_state);
}

void OdomManipulation::cmdVelCB(const geometry_msgs::TwistConstPtr& cmd_vel)
{
    current_fused_state_.updateFusedState(*cmd_vel);
}

void OdomManipulation::imuCB(const sensor_msgs::ImuConstPtr& imu)
{
    current_fused_state_.updateFusedState(imu->orientation);

    storeFusedState(current_fused_state_);

    checkFusedStateForSlip(current_fused_state_);
}

void OdomManipulation::odomCB (const nav_msgs::OdometryConstPtr& odom)
{
    nav_msgs::Odometry manipulated_odom = *odom;

    manipulated_odom.twist.twist.angular.z *= angular_factor_;
    manipulated_odom.twist.twist.linear.x *= linear_factor_;

    odom_pub_.publish(manipulated_odom);

    // Logging
    bool standing_still, turning_in_place;
    if (odom->twist.twist.linear.x < linear_speed_threshold_)
        turning_in_place = true;
    else
        turning_in_place = false;

    if (turning_in_place && (fabs(odom->twist.twist.angular.z) < angular_speed_threshold_))
        standing_still = true;
    else
        standing_still = false;

    current_fused_state_.updateFusedState(*odom, manipulated_odom);
    current_fused_state_.updateFusedState(turning_in_place, standing_still);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "odom_manipulation");

    ros::start();
    ros::Rate* const rate = new ros::Rate(60);
    OdomManipulation odom_manipulator(rate);

    ros::spin();

    delete rate;
    ros::shutdown();
    return 0;
}
