#ifndef FRANKA_JOYSTICK_CONTROL_LIB_H
#define FRANKA_JOYSTICK_CONTROL_LIB_H

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <franka_msgs/ErrorRecoveryActionGoal.h>
#include <controller_manager_msgs/SwitchController.h>

#include <joystick_msgs/Joystick.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <sensor_msgs/JointState.h>

class FrankaJoystickControl
{
  public:
    FrankaJoystickControl(ros::NodeHandle *nh);
    void joystickCallback(const joystick_msgs::Joystick::ConstPtr &msg);
    void SendCartesianVelocityCommand(const geometry_msgs::Twist cartesian_velocity_command);
    void ErrorRecovery();
    void ChangeController(const std::string start_controller,
                          const std::string stop_controller);

    // Kinematics
    void FrankaStateCallback(const sensor_msgs::JointState &msg);
    Eigen::Matrix4d DH_matrix(const double a, const double d, const double alpha, const double theta);

    struct JoystickControlCmd
    {
      double EEF_x_cmd = 0.0;             // offset of EEF in its x axis
      bool error_recovery = false;        // button for error_recovery msg
      int change_mode = 0;                //
      bool change_mode_flag = false;      // flag for being pressed
    };

    enum ControlMode
    {
      TRAJECTORY_MODE = 0,
      FORCE_MODE = 1,
    };

    // Collaborative mode
    bool collaborative_mode_flag = false;
    JoystickControlCmd joystick_cmd;

    // Kinematics variable
    Eigen::Matrix4d X_base_eef;

  private:
    const int _NUM_OF_JOINTS = 7;
    const double _EEF_INCREMENT = 0.02;

    ros::Publisher _cartesian_goal_publisher;             // publisher for cartesian velocity commands
    ros::Publisher _franka_error_recovery_publisher;      // publisher for error recovery message
    ros::ServiceClient _switch_controller_service;        // ros service for switching between trajectory controller and force control

    int ButtonClickFunction(const bool &BUTTON, bool &button_flag);

};

#endif // FRANKA_JOYSTICK_CONTROL_LIB_H
