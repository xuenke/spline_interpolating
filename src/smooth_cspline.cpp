#include "ros/ros.h"
#include<Eigen/Dense>
#include<nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <stdlib.h>
#include <time.h>
const int point_count = 50;
Eigen::MatrixXd coeff(point_count-1,4);
Eigen::VectorXd s(point_count);

bool SmoothCSpline(Eigen::VectorXd t, Eigen::VectorXd q, double mu, Eigen::MatrixXd W) {
    // check count of given points
    const int n = q.size();
    if (n != t.size())
        return false;

    coeff = Eigen::MatrixXd::Zero(n-1, 4); // coefficents of n-1 splines

    Eigen::VectorXd T(n-1); // calculate dt
    T = Eigen::VectorXd::Zero(n-1);
    for (int i = 0; i < n - 1; i++) {
        T(i) = t(i+1) - t(i);
    }

    // calculate A and C, A * omega = C * s
    Eigen::MatrixXd A(n,n);
    A = Eigen::MatrixXd::Zero(n,n);
    Eigen::MatrixXd C(n,n);
    C = A;
    for (int i = 1; i < n - 1; i++) {
        A(i,i) = 2.0 * (T(i-1) + T(i));
        C(i,i) = - 6.0 * (1.0 / T(i - 1) + 1.0 / T(i));
    }
    for (int i = 0; i < n - 1; i++) {
        A(i, i+1) = T(i);
        C(i, i+1) = 6.0 / T(i);
    }
    for (int i = 1; i < n; i ++) {
        A(i, i-1) = T(i-1);
        C(i, i-1) = 6.0 / T(i-1);
    }
    A(0,0) = 2.0 * T(0);
    C(0,0) = - 6.0 / T(0);
    A(n-1, n-1) = 2.0 * T(n-2);
    C(n-1, n-1) = - 6.0 / T(n-2);

    double lambda = (1 - mu) / mu / 6.0; 
    Eigen::VectorXd omega;
    Eigen::MatrixXd temp;
    temp = (A + lambda * C * W * C.transpose());
    omega = temp.inverse() * C * q;
    s = q - lambda * W * C.transpose() * omega;

    for (int i = 0; i < n-1; i ++) {
        coeff(i, 0) = (omega(i + 1) - omega(i)) / T(i) / 6.0;
        coeff(i, 1) = omega(i) / 2.0;
        coeff(i, 2) = (s(i + 1) - s(i)) / T(i) - 
            T(i) * (omega(i+1) + 2.0 * omega(i)) / 6.0;
        coeff(i, 3) = s(i);
    }
    for (int i = 0; i < n; i ++) {
        ROS_INFO("s%d = %lf", i, s(i));
    }
    std::cout << "A = \n" << A << std::endl;
    std::cout << "C = \n" << C << std::endl;
    std::cout << "omega = \n" << omega << std::endl;
}

void PublishPlan(
    const ros::Publisher& pub,
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(plan.size());

  if (!plan.empty()) {
    gui_path.header.frame_id = "world";
    gui_path.header.stamp = ros::Time::now();
  }

  for (unsigned int i = 0; i < plan.size(); i++) {
    gui_path.poses[i] = plan[i];
  }

  // publish
  pub.publish(gui_path);
}

void CreatePath(const Eigen::VectorXd& path_x, const Eigen::VectorXd& path_y,
                const Eigen::VectorXd& path_theta, std::vector<geometry_msgs::PoseStamped>& path) {
    
    int num = path_x.size();
    for (int i = 0; i < num; i++) {
        geometry_msgs::PoseStamped point;
        point.pose.position.x = path_x(i);
        point.pose.position.y = path_y(i);
        point.pose.position.z = 0.0;
        point.pose.orientation = tf::createQuaternionMsgFromYaw(path_theta(i));
        path.push_back(point);
        path.back().header.frame_id = "world";
        ROS_INFO("path point%d = (%lf, %lf, %lf)", i, path_x(i), path_y(i), path_theta(i));
    }
} 

void CreateRoughPath(std::vector<double>& p_x, std::vector<double>& p_y,
        std::vector<double>& p_theta) {
    double radius = 5.0;
    double max_peak = 0.1;
    double max_bias = 0.1;
    int num = 50;
    double step = M_PI / num / 2.0;
    p_x.clear();
    p_y.clear();
    p_theta.clear();
    srand(time(NULL));
    for (int i = num - 1; i >= 0; i--) {
        double rand_y = rand() % 100 / 100.0 * max_peak;
        double rand_theta = rand() % 100 / 100.0 * max_bias + M_PI;
        p_x.push_back(radius * cos(i * step));
        p_y.push_back(radius * sin(i * step)+ rand_y);
        p_theta.push_back(M_PI / 2 + i * step + rand_theta);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "splne");

    // Eigen::VectorXd q(7);
    // q << 3.0, -2.0, -5.0, 0.0, 6.0, 12.0, 8.0;
    // Eigen::VectorXd t(7);
    // t << 0.0, 5.0, 7.0, 8.0, 10.0, 15.0, 18.0;
    // int n = q.size();
    // for (int i = 0; i < n; i++) {
    //     ROS_INFO("vector q%d=%lf, t%d=%lf", i, q(i), i, t(i));
    // }
    
    // double mu = 0.3;
    // Eigen::MatrixXd w(7, 7);
    // w = Eigen::MatrixXd::Zero(7,7);
    // w(1,1) = 1.0;
    // w(2,2) = 0.1;
    // w(3,3) = 1.0;
    // w(4,4) = 1.0;
    // w(5,5) = 1.0;
    // SmoothCSpline(t, q, mu, w);
    

    // // build a plan
    // int num = 7;
    // double step = M_PI / num / 2.0;
    // double radius = 1.0;
    // Eigen::VectorXd path_x(num);
    // Eigen::VectorXd path_y(num);
    // Eigen::VectorXd path_theta(num);
    // for (int i = 0; i < num; i ++) {
    //     // path_x(num - i -1) = radius * cos(i * step);
    //     // path_y(num - i -1) = radius * sin(i * step);
    //     path_x(i) = t(i);
    //     path_y(i) = q(i);
    //     path_theta(num - i -1) = M_PI / 2 + i * step;
    // }

    // std::vector<geometry_msgs::PoseStamped> path;
    // CreatePath(path_x, path_y, path_theta, path);
    // std::vector<geometry_msgs::PoseStamped> path_smooth;
    // SmoothCSpline(path_x, path_y, mu, w);
    // CreatePath(path_x, s, path_theta, path_smooth);

    ros::Publisher plan_pub;
    ros::Publisher smooth_plan_pub;
    ros::Publisher pose_pub;
    ros::NodeHandle nh;
    plan_pub = nh.advertise<nav_msgs::Path>("plan", 1);
    smooth_plan_pub = nh.advertise<nav_msgs::Path>("smooth_plan", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

    std::vector<double> p_x, p_y, p_theta;
    CreateRoughPath(p_x, p_y, p_theta);
    const int length = p_x.size();
    Eigen::VectorXd path1_x(length);
    Eigen::VectorXd path1_y(length);
    Eigen::VectorXd path1_theta(length);
    for (int i = 0; i < length; i++) {
        path1_x(i) = p_x.at(i);
        path1_y(i) = p_y.at(i);
        path1_theta(i) = p_theta.at(i);
    }
    std::vector<geometry_msgs::PoseStamped> path1;
    CreatePath(path1_x, path1_y, path1_theta, path1);

    double mu = 0.3;
    Eigen::MatrixXd w(point_count, point_count);
    w = Eigen::MatrixXd::Zero(point_count,point_count);
    for (int i = 1; i < point_count -1; i ++) {
        w(i,i) = 1.0;
    }

    std::vector<geometry_msgs::PoseStamped> path_smooth1;
    SmoothCSpline(path1_x, path1_y, mu, w);
    CreatePath(path1_x, s, path1_theta, path_smooth1);
    
    ros::Rate rate(1);
    while (ros::ok()) {
        // PublishPlan(plan_pub, path);
        PublishPlan(plan_pub, path1);
        PublishPlan(smooth_plan_pub, path_smooth1);
        // pose_pub.publish(path[1]);
        ROS_INFO("plan published");
        ros::spinOnce();
        rate.sleep();
    }
}