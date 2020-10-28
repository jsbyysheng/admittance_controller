#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <signal.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

#include "admittance_controller/joint_trajectory.h"

#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen3/Eigen/Eigen>


using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Array<double, 6, 1> Array6d;

class admittance_control {

    public:

        admittance_control( 
            ros::NodeHandle &n, ros::Rate ros_rate,   
            std::string topic_force_sensor_subscriber, std::string topic_joint_states_subscriber,
            std::string topic_joint_trajectory_publisher, std::string topic_action_trajectory_publisher, std::string topic_joint_group_vel_controller_publisher, 
            std::vector<double> mass_model_matrix, std::vector<double> damping_model_matrix, 
            double force_dead_zone, double torque_dead_zone, double admittance_weight, std::vector<double> joint_limits,
            std::vector<double> maximum_velocity, std::vector<double> maximum_acceleration);

        ~admittance_control();

        void spinner (void);

    private:

        ros::NodeHandle nh;
        ros::Rate loop_rate;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- Admittance Parameters ---- //
        Matrix6d mass_matrix, damping_matrix;
        double force_dead_zone, torque_dead_zone, admittance_weight;
        
        // ---- Admittance IO ---- //
        Vector6d external_wrench, x_dot, q_dot;
        Vector6d q_dot_last_cycle, x_dot_last_cycle;
        
        // ---- Limits ---- //
        Vector6d joint_lim, max_vel, max_acc;

        // ---- MoveIt Robot Model ---- //
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        const robot_state::JointModelGroup *joint_model_group;
        std::vector<std::string> joint_names;
        Eigen::MatrixXd J;

        // ---- Other Variables ---- //
        bool force_callback, joint_state_callback;
        bool use_feedback_velocity, use_ur_real_robot, inertia_reduction;
        sensor_msgs::JointState joint_state;
        std::vector<double> joint_position, joint_velocity;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- PUBLISHERS & SUBSCRIBERS ---- //
        ros::Subscriber force_sensor_subscriber, joint_states_subscriber, trajectory_execution_subscriber;
        ros::Publisher joint_trajectory_publisher, joint_group_vel_controller_publisher;

        // ---- ROS SERVICES ---- //
        ros::ServiceClient switch_controller_client, list_controllers_client;
        controller_manager_msgs::SwitchController switch_controller_srv;
        controller_manager_msgs::ListControllers list_controllers_srv;
        
        // ---- ROS ACTIONS ---- //
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_client;
		control_msgs::FollowJointTrajectoryGoal trajectory_goal;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- CALLBACKS ---- //
        void force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &);
        void joint_states_Callback (const sensor_msgs::JointState::ConstPtr &);
        void trajectory_execution_Callback (const admittance_controller::joint_trajectory::ConstPtr &);

        // ---- KINEMATIC MODEL FUNCTIONS ---- //
        Eigen::Matrix4d compute_fk (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Eigen::MatrixXd compute_arm_jacobian (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Matrix6d get_ee_rotation_matrix (std::vector<double> joint_position, std::vector<double> joint_velocity);

        // ---- ADMITTANCE FUNCTIONS ---- //
        void compute_admittance (void);

        // ---- LIMIT DYNAMIC FUNCTIONS ---- //
        Vector6d limit_joint_dynamics (Vector6d joint_velocity);
        Vector6d compute_inertia_reduction (Vector6d velocity, Vector6d wrench);

        // ---- CONTROL FUNCTIONS ---- //
        void trajectory_execution (std::vector<sensor_msgs::JointState> trajectory);
        void send_velocity_to_robot (Vector6d velocity);
        void send_position_to_robot (Vector6d position);
        void wait_for_position_reached (Vector6d desired_position);

        // ---- USEFUL FUNCTIONS ---- //
        void wait_for_callbacks_initialization (void);
        int sign (double num);

//----------------------------------------------------------------------------------------------------------------------//

        // ---- DEBUG ---- //
        std::ofstream ft_sensor;

};

#endif /* ADMITTANCE_CONTROLLER_H */