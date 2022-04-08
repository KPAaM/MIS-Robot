#ifndef FRANKA_JOYSTICK_CONTROL_LIB_H
#define FRANKA_JOYSTICK_CONTROL_LIB_H

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <controller_manager_msgs/SwitchController.h>

#include <joystick_msgs/Joystick.h>


class FrankaJoystickControl
{
  public:
    FrankaJoystickControl(ros::NodeHandle *nh);

    void joystickCallback(const joystick_msgs::Joystick::ConstPtr &msg);
    void SendCartesianVelocityCommand(const geometry_msgs::Twist cartesian_velocity_command);
    geometry_msgs::Pose GetCurrentState();
    void ErrorRecovery();
    void ChangeController(const std::string start_controller,
                          const std::string stop_controller);
    void CreateGoalFrame(const double offset);



    struct JoystickControlCmd
    {
      double EEF_x_cmd = 0.0;           // offset of EEF in its x axis
      bool error_recovery = false;
      bool error_recovery_flag = false;
      int change_mode = 0;
    };

    enum ControlMode
    {
      TRAJECTORY_MODE = 0,
      FORCE_MODE = 1,
    };

    // Collaborative mode
    bool collaborative_mode_flag = false;
    JoystickControlCmd joystick_cmd;


  private:
    const int _NUM_OF_JOINTS = 7;
    const double _EEF_INCREMENT = 0.25;

    ros::Publisher _cartesian_goal_publisher;             // publisher for cartesian velocity commands
    ros::Publisher _franka_error_recovery_publisher;      // publisher for error recovery message
    ros::ServiceClient _switch_controller_service;        // ros service for switching between trajectory controller and force control

    // Moveit
    moveit::planning_interface::MoveGroupInterface _group;
    robot_model_loader::RobotModelLoader _robot_model_loader;
    robot_model::RobotModelPtr _robot_model_ptr;
    robot_state::RobotStatePtr _goal_state_ptr;
    const robot_state::JointModelGroup* _joint_model_group;

    // TF2 listener for goal_frame
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;
};

#endif // FRANKA_JOYSTICK_CONTROL_LIB_H
