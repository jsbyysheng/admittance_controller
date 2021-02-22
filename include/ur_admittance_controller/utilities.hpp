//
// Created by robotmws on 2021/2/21.
//

#ifndef UR_ADMITTANCE_CONTROLLER_UTILITIES_HPP
#define UR_ADMITTANCE_CONTROLLER_UTILITIES_HPP

#define PACKAGE_NAME "ur_admittance_controller"

#include <csignal>
#include <fstream>
#include <iostream>
#include <cstring>
#include <numeric>
#include <cmath>

#include "spline_interpolation/spline.h" /* https://kluge.in-chemnitz.de/opensource/spline/ */

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "ur_admittance_controller/parameter_msg.h"
#include "ur_admittance_controller/joint_trajectory.h"
#include "ur_admittance_controller/parameter_srv.h"

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Eigen>

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, 6, 1> Array6d;

struct extra_data_keypoint {
    double time_keypoint;
    double data_value;
};

template<typename T> void loadParam(const ros::NodeHandle &nodeHandle, const std::string &key, T &var) {
    if (!nodeHandle.getParam(key, var)) {
        ROS_ERROR("param %s cannot be loaded.", key.c_str());
        ros::shutdown();
    }
    std::ostringstream oss;
    oss << var;
    ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
}

template<typename T> void loadParam(const ros::NodeHandle &nodeHandle, const std::string &key, std::vector<T> &var) {
    if (!nodeHandle.getParam(key, var)) {
        ROS_ERROR("param %s cannot be loaded.", key.c_str());
        ros::shutdown();
    }
    std::ostringstream oss;
    for(size_t i = 0; i < var.size(); ++i) {
        if(i != 0) oss << ",";
        oss << var[i];
    }
    ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
}

template<typename T> void loadParam(const ros::NodeHandle &nodeHandle, const std::string &key, T &var, const T &defaultVal) {
    nodeHandle.param(key, var, defaultVal);
    std::ostringstream oss;
    oss << var;
    ROS_DEBUG("param %s is %s", key.c_str(), oss.str().c_str());
}

inline void csv_debug(const std::vector<double>& vector, const std::string& name) {
    std::string package_path = ros::package::getPath(PACKAGE_NAME);
    std::string save_file = package_path + "/debug/" + name + "_debug.csv";
    std::ofstream vector_debug = std::ofstream(save_file);
    vector_debug << name << "\n";
    for (double i : vector) {
        vector_debug << i << "\n";
    }
    vector_debug.close();

}

inline void csv_debug(std::vector<Vector6d> vector6d, const std::string& name) {
    std::string package_path = ros::package::getPath(PACKAGE_NAME);
    std::string save_file = package_path + "/debug/" + name + "_debug.csv";
    std::ofstream vector_debug = std::ofstream(save_file);
    vector_debug << name << "\n";
    for (auto & i : vector6d) {
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {
            vector_debug << i[joint_n] << ",";
        }
        vector_debug << "\n";
    }
    vector_debug.close();
}

inline void csv_debug(std::vector<tk::spline> spline6d, std::vector<double> s, const std::vector<Vector6d>& data_vector, const std::string& name) {
    std::string package_path = ros::package::getPath(PACKAGE_NAME);
    std::string save_file = package_path + "/debug/" + name + "_spline6d_debug.csv";
    std::ofstream spline6d_debug = std::ofstream(save_file);
    spline6d_debug << "s, ,Joint1,Joint2,Joint3,Joint4,Joint5,Joint6\n\n";
    for (unsigned int i = 0; i < data_vector.size(); i++) {
        spline6d_debug << s[i] << ", ,";
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) { spline6d_debug << spline6d[joint_n](s[i]) << ","; }
        spline6d_debug << "\n";
    }
    spline6d_debug.close();
}

inline void trajectory_debug_csv(std::vector<sensor_msgs::JointState> trajectory, const std::string& trajectory_name) {
    std::string package_path = ros::package::getPath(PACKAGE_NAME);
    std::string save_file = package_path + "/debug/" + trajectory_name + "_debug.csv";
    std::ofstream trajectory_debug = std::ofstream(save_file);
    trajectory_debug
            << "frame_id,seq,sec,nsec,     ,pos_joint1,pos_joint2,pos_joint2,pos_joint4,pos_joint5,pos_joint6,     ,vel_joint1,vel_joint2,vel_joint3,vel_joint4,vel_joint5,vel_joint6\n\n";
    for (auto & i : trajectory) {
        trajectory_debug << i.header.frame_id << "," << i.header.seq << ","
                         << i.header.stamp.sec << "," << i.header.stamp.nsec << ", ,";
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {
            trajectory_debug << i.position[joint_n] << ",";
        }
        trajectory_debug << " ,";
        for (unsigned int joint_n = 0; joint_n < 6; joint_n++) {
            trajectory_debug << i.velocity[joint_n] << ",";
        }
        trajectory_debug << "\n";
    }
    trajectory_debug.close();
}

inline int sign(double num) {
    if (num >= 0) return 1;
    else return -1;
}

inline std::vector<tk::spline> spline_interpolation(std::vector<Vector6d> data_vector, double spline_lenght, const std::string& output_file, bool debug=false) {
    // Creation of Spline6d Vector -> Usage: spline6d[joint_number](s) = q(s)
    std::vector<tk::spline> spline6d;
    // Creation of s € [0,spline_lenght] vector
    std::vector<double> s;
    for (unsigned i = 0; i < data_vector.size(); i++) {
        double s_i = spline_lenght * (i / (double(data_vector.size()) - 1));
        s.push_back(s_i);
    }
    // Compute Spline for each Joint
    for (unsigned joint_number = 0; joint_number < 6; joint_number++) {
        // Create a Single-Joint Vector
        std::vector<double> waypoints_1d;
        for (auto & i : data_vector) {
            waypoints_1d.push_back(i[joint_number]);
        }
        // Compute Cubic Spline [Q(s), s € [0,T]]
        tk::spline spline1d;
        spline1d.set_points(s, waypoints_1d);
        // Add Results to "spline6d" Vector
        spline6d.push_back(spline1d);
    }
    // ---- DEBUG ---- //
    if (debug) {
        csv_debug(spline6d, s, data_vector, output_file);
    }
    return spline6d;
}

inline std::vector<tk::spline> spline_interpolation(std::vector<Array6d> data_vector, double spline_lenght, const std::string& output_file) {
    // Converting Array into Vector
    std::vector<Vector6d> data_vector_temp;
    for (auto & i : data_vector) {
        data_vector_temp.emplace_back(i.matrix());
    }
    return spline_interpolation(data_vector_temp, spline_lenght, output_file);
}

inline Vector6d get_spline_value(std::vector<tk::spline> spline6d, double s) {
    Vector6d spline_value;
    spline_value.resize(6);

    // Get spline1d value for each joint
    for (unsigned int i = 0; i < 6; i++) {
        spline_value[i] = spline6d[i](s);
    }
    return spline_value;
}

inline extra_data_keypoint new_extra_data_keypoint(double data_value, double time_keypoint) {
    extra_data_keypoint temp{};
    temp.data_value = data_value;
    temp.time_keypoint = time_keypoint;
    return temp;
}

inline Vector6d new_vector_6d(double x, double y, double z, double roll, double pitch, double yaw) {
    Vector6d temp;
    temp.setZero();
    temp[0] = x;
    temp[1] = y;
    temp[2] = z;
    temp[3] = roll;
    temp[4] = pitch;
    temp[5] = yaw;
    return temp;
}

#endif //UR_ADMITTANCE_CONTROLLER_UTILITIES_HPP
