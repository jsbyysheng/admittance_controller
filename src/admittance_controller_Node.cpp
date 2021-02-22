#include "ur_admittance_controller/admittance_controller.h"


void Shutdown_Signal_Handler (int sig) {
    ros::NodeHandle nh;
    
    std::string topic_joint_group_vel_controller_publisher;
    nh.param<std::string>("/admittance_controller_Node/topic_joint_group_vel_controller", topic_joint_group_vel_controller_publisher, "/joint_group_vel_controller/command");

    ros::Publisher joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);

    std_msgs::Float64MultiArray stop_msgs;
    std::vector<double> stop_vector;
    stop_vector.resize(6, 0.0);

    stop_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
    stop_msgs.layout.dim[0].size = stop_vector.size();
    stop_msgs.layout.dim[0].stride = 1;
    stop_msgs.layout.dim[0].label = "velocity";

    // copy in the data
    stop_msgs.data.clear();
    stop_msgs.data.insert(stop_msgs.data.end(), stop_vector.begin(), stop_vector.end());

    joint_group_vel_controller_publisher.publish(stop_msgs);

    ros::shutdown();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "admittance_controller_Node", ros::init_options::NoSigintHandler);

    signal(SIGINT, Shutdown_Signal_Handler);

    ros::NodeHandle nh;
    ros::Rate loop_rate = 1000;

    /**
     * sensor data
     * 1. force/torque sensor
     * 2. robot joint state
     * **/
    // state topic: force_torque_sensor
    std::string topic_force_sensor_subscriber;
    // state topic: joint_states topic
    std::string topic_joint_states_subscriber;

    /**
     * control model (interfaces required)
     * 1. free drive: using /ur_hardware_interface/script_command to call URScript func freedrive_mode()
     *    and end_freedrive_mode(). This is buildin free drive.
     *
     * 2. admittance controller:
     *
     * 3. moveit:
     *
     * **/
    // ur control topic (command interface, position_controllers)
    std::string topic_joint_trajectory_publisher;
    // ur control topic (moveit interface, position_controllers)
    std::string topic_action_trajectory_publisher;
    // ur control topic (moveit interface, velocity_controllers)
    std::string topic_joint_group_vel_controller_publisher;

    /**
     * admitance control M * xdd + D * xd + S * x = F_ext
     * factors: M: mass D: damping S: stiffness
     * **/
    std::vector<double> mass_model_matrix;
    std::vector<double> damping_model_matrix;
    double force_dead_zone;
    double torque_dead_zone;
    double admittance_weight;

    /**
     * some safety limits
     * **/
    std::vector<double> joint_limits;
    std::vector<double> maximum_velocity;
    std::vector<double> maximum_acceleration;

    // ---- LOADING "TOPIC NAME" PARAMETERS FROM THE ROS SERVER ---- //
    ros::NodeHandle paramNodeHandle(ros::this_node::getName());

    /** ---- LOADING STATES TOPICS FROM THE ROS SERVER ---- **/
    loadParam(paramNodeHandle, std::string("topic_force_sensor"), topic_force_sensor_subscriber, std::string("/wrench"));
    loadParam(paramNodeHandle, std::string("topic_joint_states"), topic_joint_states_subscriber, std::string("/joint_states"));

    /** ---- LOADING ROBOT CONTROL TOPICS PARAMETERS FROM THE ROS SERVER ---- **/
    loadParam(paramNodeHandle, std::string("topic_joint_trajectory"), topic_joint_trajectory_publisher, std::string("/scaled_pos_joint_traj_controller/command"));
    loadParam(paramNodeHandle, std::string("topic_action_trajectory"), topic_action_trajectory_publisher, std::string("/scaled_pos_joint_traj_controller/follow_joint_trajectory"));
    loadParam(paramNodeHandle, std::string("topic_joint_group_vel_controller"), topic_joint_group_vel_controller_publisher, std::string("/joint_group_vel_controller/command"));

    /** ---- LOADING ADMITTANCE PARAMETERS FROM THE ROS SERVER ---- **/
    loadParam(paramNodeHandle, std::string("mass_matrix"), mass_model_matrix);
    loadParam(paramNodeHandle, std::string("damping_matrix"), damping_model_matrix);
    loadParam(paramNodeHandle, std::string("force_dead_zone"), force_dead_zone, 3.0);
    loadParam(paramNodeHandle, std::string("torque_dead_zone"), torque_dead_zone, 1.0);
    loadParam(paramNodeHandle, std::string("admittance_weight"), admittance_weight, 1.0);

    /** ---- LOADING "SAFETY" PARAMETERS FROM THE ROS SERVER ---- **/
    loadParam(paramNodeHandle, std::string("joint_limits"), joint_limits);
    loadParam(paramNodeHandle, std::string("maximum_velocity"), maximum_velocity);
    loadParam(paramNodeHandle, std::string("maximum_acceleration"), maximum_acceleration);
    

    auto *ac = new admittance_control (
        nh,
        loop_rate,
        topic_force_sensor_subscriber,
        topic_joint_states_subscriber,
        topic_joint_trajectory_publisher,
        topic_action_trajectory_publisher,
        topic_joint_group_vel_controller_publisher,
        mass_model_matrix,
        damping_model_matrix,
        force_dead_zone,
        torque_dead_zone,
        admittance_weight,
        joint_limits,
        maximum_velocity,
        maximum_acceleration
    );

    while (ros::ok()) ac -> spinner();
    delete ac;

    return 0;
}
