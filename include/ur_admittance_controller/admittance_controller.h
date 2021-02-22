#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "utilities.hpp"

#define GET_VARIABLE_NAME(Variable) (#Variable)

class admittance_control {

    public:

        admittance_control( 
            ros::NodeHandle &n,
            ros::Rate ros_rate,
            std::string topic_force_sensor_subscriber,
            std::string topic_joint_states_subscriber,
            std::string topic_joint_trajectory_publisher,
            std::string topic_action_trajectory_publisher,
            std::string topic_joint_group_vel_controller_publisher,
            std::vector<double> mass_model_matrix,
            std::vector<double> damping_model_matrix,
            double force_dead_zone,
            double torque_dead_zone,
            double admittance_weight,
            std::vector<double> joint_limits,
            std::vector<double> maximum_velocity,
            std::vector<double> maximum_acceleration
        );

        ~admittance_control();

        void spinner();

        bool simple_debug;
        bool complete_debug;

    private:

        ros::NodeHandle nh;
        ros::Rate loop_rate;

        // ---- Admittance Parameters ---- //
        Matrix6d mass_matrix, damping_matrix;
        double force_dead_zone, torque_dead_zone, admittance_weight;
        bool use_feedback_velocity, use_ur_real_robot, inertia_reduction;
        
        // ---- Admittance IO ---- //
        Vector6d external_wrench, ftsensor_start_offset, x_dot, q_dot;
        Vector6d q_dot_last_cycle, x_dot_last_cycle;
        
        // ---- Limits ---- //
        Vector6d joint_limit, max_vel, max_acc;

        // ---- MoveIt Robot Model ---- //
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        const robot_state::JointModelGroup *joint_model_group;
        std::vector<std::string> joint_names;
        Eigen::MatrixXd J;

        // ---- Trajectory Execution ---- //
        ur_admittance_controller::joint_trajectory desired_trajectory;
        std::vector<extra_data_keypoint> velocity_extra_data_keypoint, force_extra_data_keypoint;

        // ---- Feedback Variables ---- //
        bool force_callback, joint_state_callback;
        sensor_msgs::JointState joint_state;
        std::vector<double> joint_position, joint_velocity;
        std::vector<Vector6d> filter_elements;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- PUBLISHERS & SUBSCRIBERS ---- //
        ros::Subscriber force_sensor_subscriber, joint_states_subscriber, trajectory_execution_subscriber;
        ros::Publisher joint_trajectory_publisher, joint_group_vel_controller_publisher, ur_script_command_publisher;
        ros::Publisher cartesian_position_publisher;

        // ---- ROS SERVICE CLIENTS ---- //
        ros::ServiceClient switch_controller_client;
        ros::ServiceClient list_controllers_client;
        ros::ServiceClient ur_zero_ft_sensor_client;
        ros::ServiceClient ur_resend_robot_program;
        ros::ServiceClient ur_play_urcap;
        controller_manager_msgs::SwitchController switch_controller_srv;
        controller_manager_msgs::ListControllers list_controllers_srv;
        std_srvs::Trigger ur_zero_ft_sensor_srv;
        std_srvs::Trigger ur_resend_robot_program_srv;
        std_srvs::Trigger ur_play_urcap_srv;
        
        // ---- ROS SERVICE SERVERS ---- //
        ros::ServiceServer admittance_controller_activation_service, change_admittance_parameters_service, ur_freedrive_mode_service, ur_restart_urcap_service;
        bool admittance_control_request, freedrive_mode_request, trajectory_execution_request;

        // ---- ROS ACTIONS ---- //
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trajectory_client;
        control_msgs::FollowJointTrajectoryGoal trajectory_goal;

//----------------------------------------------------------------------------------------------------------------------//

        // ---- CALLBACKS ---- //
        void force_sensor_Callback (const geometry_msgs::WrenchStamped::ConstPtr &);
        void joint_states_Callback (const sensor_msgs::JointState::ConstPtr &);
        void trajectory_execution_Callback (const ur_admittance_controller::joint_trajectory::ConstPtr &);

        // ---- SERVER CALLBACKS ---- //
        bool Admittance_Controller_Activation_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool Change_Admittance_Parameters_Service_Callback (ur_admittance_controller::parameter_srv::Request &req, ur_admittance_controller::parameter_srv::Response &res);
        bool FreedriveMode_Service_Callback (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool Restart_URCap_Service_Callback (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        // ---- KINEMATIC MODEL FUNCTIONS ---- //
        Eigen::Matrix4d compute_fk ();
        Eigen::MatrixXd compute_arm_jacobian (std::vector<double> joint_position, std::vector<double> joint_velocity);
        Matrix6d get_ee_rotation_matrix (std::vector<double> joint_position, std::vector<double> joint_velocity);

        // ---- ADMITTANCE FUNCTIONS ---- //
        void compute_admittance ();

        // ---- LIMIT DYNAMIC FUNCTIONS ---- //
        Vector6d limit_joint_dynamics (Vector6d jnt_velocity);
        Vector6d compute_inertia_reduction (Vector6d velocity, Vector6d wrench);

        // ---- TRAJECTORY FUNCTIONS ---- //
        void trajectory_execution (ur_admittance_controller::joint_trajectory desired_trajectory);
        void apply_force (double force_value);
        void stop_robot ();
        sensor_msgs::JointState add_stop_point (std::vector<sensor_msgs::JointState> *trajectory);

        // ---- TRAJECTORY SCALING ---- //
        std::vector<sensor_msgs::JointState> trajectory_scaling (ur_admittance_controller::joint_trajectory trajectory);
        void check_requested_scaling (ur_admittance_controller::joint_trajectory *trajectory);
        double compute_scaled_velocity (ur_admittance_controller::joint_trajectory trajectory, double s_dot_rec);
        std::vector<double> compute_s_des (double s_dot_des, double trajectory_time, double sampling_time, std::vector<ur_admittance_controller::parameter_msg> extra_data);
        std::vector<Vector6d> compute_desired_positions (std::vector<double> s_des, std::vector<tk::spline> q_spline6d);
        std::vector<Vector6d> compute_desired_velocities (std::vector<Vector6d> q_des, double sampling_time);
        std::vector<sensor_msgs::JointState> create_scaled_trajectory (std::vector<sensor_msgs::JointState> input_trajectory, std::vector<Vector6d> q_des, std::vector<Vector6d> q_dot_des, double sampling_time);

        // ---- CONTROL FUNCTIONS ---- //
        void send_velocity_to_robot (Vector6d velocity);
        void send_position_to_robot (Vector6d position);
        void wait_for_position_reached (Vector6d desired_position, double maximum_time);

        // ---- UR FUNCTIONS ---- //
        void wait_for_callbacks_initialization () const;
        void ur_zero_ft_sensor ();
        void ur_send_script_command (std::string command);
        void start_freedrive_mode ();
        void stop_freedrive_mode ();
        void ur_restart_urcap ();

        // ---- USEFUL FUNCTIONS ---- //
        void publish_cartesian_position();
        Vector6d low_pass_filter(const Vector6d& input_vec);

        // ---- DEBUG ---- //
        std::ofstream ft_sensor_debug;
};

#endif /* ADMITTANCE_CONTROLLER_H */
