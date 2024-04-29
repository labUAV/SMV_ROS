/**
 * \brief Trajectory planner based on an Artificial Potential Field (APF)
 * This ROS1 node receives an odometry message containing the robot estimated
 * states and computes a reference for the controller. The APF gives a 
 * reference velocity and heading.
 * 
 * @authors Davide Carminati, Iris David Du Mutel de Pierrrepont Franzetti
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

#include "planner_controller/APFTrajectory.h"

using namespace Eigen;

class APF
{
    private:
        MatrixXd pdist2(MatrixXd points1, MatrixXd points2);
        ros::NodeHandle nh;
        ros::Rate loop_rate;
        ros::Publisher trajectory_publisher;
        ros::Subscriber odometry_subscriber;
        Vector2d goal_position;
        MatrixXd obstacle_position;
        int number_obstacles;
        double kp;                              // Attractive gain
        double rho_0;                           // Critical radius
        double eta;                             // Repulsive gain
        double goal_tol;                        // Tolerance on reaching the goal
    public:
        APF(ros::NodeHandle* nh, double freq);
        ~APF();
        void callback(const nav_msgs::Odometry::ConstPtr &estimated_states);
};

APF::APF(ros::NodeHandle* nh, double freq) : nh(*nh), loop_rate(freq)
{
    trajectory_publisher = nh->advertise<planner_controller::APFTrajectory>("/trajectory", 1);

    // Retrieving goal and obstacle positions
    if (!ros::param::get("/goal_position_x", this->goal_position(0)))
        goal_position(0) = 1.0;
    if (!ros::param::get("/goal_position_y", this->goal_position(1)))
        goal_position(1) = 1.0;

    if (!ros::param::get("/number_obstacles", number_obstacles))
        number_obstacles = 0;

    if (number_obstacles > 0)
    {
        std::vector<double> tmp_obstacle_position(2 * number_obstacles);
        if (!ros::param::get("/obstacle_position", tmp_obstacle_position))
        {
            ROS_ERROR("Invalid obstacle coordinates. Check the launch file.");
            return;
        }
        obstacle_position.resize(number_obstacles, 2);
        obstacle_position = Map<MatrixXd>(tmp_obstacle_position.data(), number_obstacles, 2);
    }

    // Retrieving APF parameters
    if (!ros::param::get("/kp", kp))
        kp = 1.0;
    if (!ros::param::get("/rho_0", rho_0))
        rho_0 = 0.3;
    if (!ros::param::get("/eta", eta))
        eta = 0.5;
    if (!ros::param::get("/goal_tolerance", goal_tol))
        goal_tol = 0.05;

    // Creating subscriber to estimated states
    odometry_subscriber = nh->subscribe("/estimated_state", 10, &APF::callback, this);
}

APF::~APF()
{
    //
}

void APF::callback(const nav_msgs::Odometry::ConstPtr &estimated_states)
{
    // Enter here each time a new odometry message is available in the subscribed topic "/estimated_states"
    Vector2d estimated_position;
    estimated_position <<   estimated_states->pose.pose.position.x, 
                            estimated_states->pose.pose.position.y;

    // Compute robot-goal distance
    double goal_distance = (estimated_position - goal_position).norm();

    if (goal_distance <= goal_tol)
    {
        // Robot reached the goal. No new reference is published.
        ROS_INFO("Goal reached. Goal distance: %fm", goal_distance);
        return;
    }

    // Attractive contribution
    double d = 2.0;
    double F_attractive_x, F_attractive_y;
    if (goal_distance <= d)
    {
        F_attractive_x = -kp * (estimated_position(0) - goal_position(0));
        F_attractive_y = -kp * (estimated_position(1) - goal_position(1));
    }
    else
    {
        F_attractive_x = -kp * (estimated_position(0) - goal_position(0)) * d / goal_distance;
        F_attractive_y = -kp * (estimated_position(1) - goal_position(1)) * d / goal_distance;
    }

    // Repulsive contribution
    double F_repulsive_x = 0, F_repulsive_y = 0;
    if (number_obstacles > 0)
    {
        // Compute robot-obstacles positions
        VectorXd obstacle_distance = pdist2(obstacle_position, estimated_position.transpose());
        for (int idx = 0; idx < number_obstacles; idx++)
        {
            if (obstacle_distance(idx) <= rho_0)
            {
                // The robot is inside the critical obstacle area!
                F_repulsive_x += eta * (1 / obstacle_distance(idx) - 1/rho_0) * 
                    pow(obstacle_distance(idx), -2) * (estimated_position(0) - obstacle_position(idx, 0)) / obstacle_distance(idx);
                F_repulsive_y += eta * (1 / obstacle_distance(idx) - 1/rho_0) * 
                    pow(obstacle_distance(idx), -2) * (estimated_position(1) - obstacle_position(idx, 1)) / obstacle_distance(idx);
                
            }
        }
    }

    // Adding all contributions
    double F_total_x = F_attractive_x + F_repulsive_x;
    double F_total_y = F_attractive_y + F_repulsive_y;
    Vector2d F_total;
    F_total << F_total_x, F_total_y;

    // Computing reference
    double psi_reference = atan2(F_total_y, F_total_x);
    // ROS_INFO("Psi ref: %f", psi_reference / M_PI * 180);
    double V_reference = F_total.norm() * 2;

    planner_controller::APFTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.velocity     = V_reference;
    trajectory_msg.angle.w      = cos(psi_reference / 2.0);
    trajectory_msg.angle.z      = sin(psi_reference / 2.0);

    trajectory_publisher.publish(trajectory_msg);
}

MatrixXd APF::pdist2(MatrixXd points1, MatrixXd points2)
{
    // From: https://stackoverflow.com/questions/35273292/eigen-calculate-the-distance-matrix-between-2-sets-of-vectors
    int numPoints1 = points1.rows();
    int numPoints2 = points2.rows();
    MatrixXd dist(numPoints1, numPoints2);
    for (int s = 0; s < numPoints2; s++)
    {
        dist.col(s) =
            (points1.rowwise() - points2.row(s)).matrix().rowwise().norm();
    }
    return dist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "APF");
    ros::NodeHandle n;
    double node_frequency = 20; // Hz
    APF apf(&n, node_frequency);

    ros::spin();

    return 0;
}
