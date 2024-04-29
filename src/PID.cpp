/**
 * \brief A simple, two-channel PID controller for a ground robot.
 * 
 * This ROS1 node receives an odometry message containing the estimated robot states
 * and a trajectory message from a simple APF. The controller generates 
 * a velocity command for the left and right motor of the robot.
 * 
 * @author Davide Carminati
 * Copyright (C) 2024 Davide Carminati
 * 
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <https://www.gnu.org/licenses/>. 
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "planner_controller/APFTrajectory.h"
#include "planner_controller/WheelCommand.h"

using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<planner_controller::APFTrajectory,
        nav_msgs::Odometry> MySyncPolicy;

typedef struct 
{
    double Kp = 0.0, Kd = 0.0, Ki = 0.0;
    double max_cmd = INFINITY;     // Saturation (max command)
    double min_cmd = -INFINITY;    // Saturation (min command)
} PIDParameter;

typedef struct 
{
    double integral_state = 0;
    double error_old = 0;
} PIDState;

class PID
{
private:
    ros::NodeHandle nh;
    ros::Rate loop_rate;
    PIDParameter vel_params, angle_params;
    ros::Publisher command_publisher;
    message_filters::Subscriber<planner_controller::APFTrajectory> trajectory_subscriber;
    message_filters::Subscriber<nav_msgs::Odometry> odometry_subscriber;
    MySyncPolicy policy;
    message_filters::Synchronizer<MySyncPolicy> sync;
    PIDState pid_state_vel, pid_state_psi;
    uint64_t old_timestamp = ros::Time::now().toNSec();
public:
    PID(ros::NodeHandle* nh, double freq);
    ~PID();
    void compute_command(   const planner_controller::APFTrajectory::ConstPtr &trajectory,
                            const nav_msgs::Odometry::ConstPtr &estimated_state);
};

/**
 * \brief Constructor
*/
PID::PID(ros::NodeHandle* nh, double freq) :    nh(*nh), 
                                                loop_rate(freq),
                                                trajectory_subscriber(*nh, "/trajectory", 10),
                                                odometry_subscriber(*nh, "/estimated_state", 10),
                                                policy(1),
                                                sync(static_cast<const MySyncPolicy&>(policy), trajectory_subscriber, odometry_subscriber)

{
    // Using message filters to synchronize messages coming at different frequency
    sync.registerCallback(boost::bind(&PID::compute_command, this, _1, _2));

    // Initilizing publisher
    command_publisher = this->nh.advertise<planner_controller::WheelCommand>("/command", 1);

    // Retrieving PID parameters for velocity channel
    if (!ros::param::get("/Kp_vel", vel_params.Kp))
    {
        ROS_WARN_STREAM("PID parameter /Kp_vel is set to 0 as it cannot be found in the launch file.");
        vel_params.Kp = 0.0;
    }
    if (!ros::param::get("/Ki_vel", vel_params.Ki))
    {
        ROS_WARN_STREAM("PID parameter /Ki_vel is set to 0 as it cannot be found in the launch file.");
        vel_params.Ki = 0.0;
    }
    if (!ros::param::get("/Kd_vel", vel_params.Kd))
    {
        ROS_WARN_STREAM("PID parameter /Kd_vel is set to 0 as it cannot be found in the launch file.");
        vel_params.Kd = 0.0;
    }
    if (!ros::param::get("/max_cmd_vel", vel_params.max_cmd))
    {
        vel_params.max_cmd = 200 / 60 * 2 * M_PI;
        ROS_WARN_STREAM("PID parameter /max_cmd_vel is set to " << vel_params.max_cmd << " as it cannot be found in the launch file.");
    }
    if (!ros::param::get("/min_cmd_vel", vel_params.min_cmd))
    {
        vel_params.min_cmd = -200 / 60 * 2 * M_PI;
        ROS_WARN_STREAM("PID parameter /min_cmd_vel is set to " << vel_params.min_cmd << " as it cannot be found in the launch file.");
    }

    // Retrieving PID parameters for heading channel
    if (!ros::param::get("/Kp_psi", angle_params.Kp))
    {
        ROS_WARN_STREAM("PID parameter /Kd_psi is set to 0 as it cannot be found in the launch file.");
        angle_params.Kp = 0.0;
    }
    if (!ros::param::get("/Ki_psi", angle_params.Ki))
    {
        ROS_WARN_STREAM("PID parameter /Ki_psi is set to 0 as it cannot be found in the launch file.");
        angle_params.Ki = 0.0;
    }
    if (!ros::param::get("/Kd_psi", angle_params.Kd))
    {
        ROS_WARN_STREAM("PID parameter /Kd_psi is set to 0 as it cannot be found in the launch file.");
        angle_params.Kd = 0.0;
    }
    if (!ros::param::get("/max_cmd_psi", angle_params.max_cmd))
    {
        angle_params.max_cmd = INFINITY;
        ROS_WARN_STREAM("PID parameter /max_cmd_psi is set to " << angle_params.max_cmd << " as it cannot be found in the launch file.");
    }
    if (!ros::param::get("/min_cmd_psi", angle_params.min_cmd))
    {
        angle_params.min_cmd = -INFINITY;
        ROS_WARN_STREAM("PID parameter /min_cmd_psi is set to " << angle_params.min_cmd << " as it cannot be found in the launch file.");
    }
}

PID::~PID()
{
}

/**
 * \brief Callback
*/
void PID::compute_command(  const planner_controller::APFTrajectory::ConstPtr &trajectory,
                            const nav_msgs::Odometry::ConstPtr &estimated_state)
{
    // Creating empty message
    planner_controller::WheelCommand command_msg;
    // Velocity channel
    double vel_ref = trajectory->velocity;
    double vel_est =    sqrt( pow(estimated_state->twist.twist.linear.x, 2) + 
                        pow(estimated_state->twist.twist.linear.y, 2) );
    double error_vel = vel_ref - vel_est;
    double delta_t = (ros::Time::now().toNSec() - old_timestamp) * 1e-9; // in seconds

    double p_cmd = vel_params.Kp * error_vel;
    pid_state_vel.integral_state += error_vel * delta_t;
    double i_cmd = vel_params.Ki * pid_state_vel.integral_state;
    double d_cmd = vel_params.Kd * (error_vel - pid_state_vel.error_old) / delta_t;

    double vel_cmd = p_cmd + i_cmd + d_cmd;
    // Saturate command
    if (vel_cmd > vel_params.max_cmd)
        vel_cmd = vel_params.max_cmd;
    if (vel_cmd < vel_params.min_cmd)
        vel_cmd = vel_params.min_cmd;

    // Heading channel

    /**
     * Transform a quaternion in its 4x4 matrix representation
    */
    auto quaternionAsMatrix = [](const Vector4d &a)
    {
        Matrix4d mat;
        mat <<  a(0), -a(1), -a(2), -a(3),
                a(1), a(0), -a(3), a(2),
                a(2), a(3), a(0), -a(1),
                a(3), -a(2), a(1), a(0);
        return mat;
    };

    /**
     * Compute the error quaternion defined as p x q⁻¹. p and q are vector quaternions defined as [w, x, y, z].
    */
    auto errorQuaternion = [&quaternionAsMatrix](const Vector4d &p, const Vector4d &q)
    {
        Vector4d q_conj(q(0), -q(1), -q(2), -q(3));
        Vector4d out = quaternionAsMatrix(p) * q_conj;
        if (out.norm() <= 1e-12)
        {
            out << 1.0, 0.0, 0.0, 0.0;
        }
        else
        {
            out = out / out.norm();
        }
        return out;
    };

    Vector4d psi_ref( trajectory->angle.w,
                                trajectory->angle.x,
                                trajectory->angle.y,
                                trajectory->angle.z);
    Vector4d psi_est( estimated_state->pose.pose.orientation.w,
                                estimated_state->pose.pose.orientation.x,
                                estimated_state->pose.pose.orientation.y,
                                estimated_state->pose.pose.orientation.z);

    // ROS_INFO_STREAM("psi_ref:\n" << psi_ref << "\npsi_est:\n" << psi_est);
    double error_psi = errorQuaternion(psi_ref, psi_est).coeff(3);
    // ROS_INFO_STREAM("error quaternion:\n" << error_psi );

    p_cmd = angle_params.Kp * error_psi;
    pid_state_psi.integral_state += error_psi * delta_t;
    i_cmd = angle_params.Ki * pid_state_psi.integral_state;
    d_cmd = angle_params.Kd * (error_psi - pid_state_psi.error_old) / delta_t;

    double psi_cmd = p_cmd + i_cmd + d_cmd;
    // Saturate command
    if (psi_cmd > angle_params.max_cmd)
        psi_cmd = angle_params.max_cmd;
    if (psi_cmd < angle_params.min_cmd)
        psi_cmd = angle_params.min_cmd;

    command_msg.header.stamp    = ros::Time::now();
    command_msg.omega_r         = vel_cmd + psi_cmd;
    command_msg.omega_l         = vel_cmd - psi_cmd;

    command_publisher.publish(command_msg);
        
    old_timestamp = ros::Time::now().toNSec();
    pid_state_vel.error_old = error_vel;
    pid_state_psi.error_old = error_psi;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "PID");
    ros::NodeHandle n;
    double node_frequency = 50; // Hz
    PID pid(&n, node_frequency);

    ros::spin();

    return 0;
}