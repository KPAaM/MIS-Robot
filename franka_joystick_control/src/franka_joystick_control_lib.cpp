#include "franka_joystick_control/franka_joystick_control_lib.h"
#include <functional>


///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::FrankaJoystickControl Initialization of moveit and ros communication features
/// \param nh ros::NodeHandle pointer
///////////////////////////////////////////////////////////////////////
FrankaJoystickControl::FrankaJoystickControl(ros::NodeHandle *nh)
  : _group("panda_arm"),
    _robot_model_loader("robot_description"),
    _goal_state_ptr(_group.getCurrentState()),
    _tfListener(_tfBuffer)
{
  _cartesian_goal_publisher = nh->advertise<geometry_msgs::Twist>("/joystick_cartesian_goal", 1);
  _franka_error_recovery_publisher = nh->advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
  _switch_controller_service = nh->serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

  // Moveit initialization
  _robot_model_ptr = _robot_model_loader.getModel();
  _joint_model_group = _robot_model_ptr->getJointModelGroup(_group.getName());
}


///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::joystickCallback
/// \param msg Contains signals from the joystick
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::joystickCallback(const joystick_msgs::Joystick::ConstPtr &msg)
{
  joystick_cmd.EEF_x_cmd  = _EEF_INCREMENT*(-msg->axis_2/32767);
  CreateGoalFrame(joystick_cmd.EEF_x_cmd);

  // If statement to avoid multiple recovery requests
  if (!joystick_cmd.error_recovery) joystick_cmd.error_recovery = msg->button_3;

  //ButtonClickFunction(msg->button_3, joystick_cmd.error_recovery_flag);
  if (ButtonClickFunction(msg->button_1, joystick_cmd.change_mode_flag) == 1)
  {
    if(joystick_cmd.change_mode == 1)
    {
      joystick_cmd.change_mode = 0;
    }
    else
    {
      joystick_cmd.change_mode = 1;
    }
  }
}

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::CreateGoalFrame Create a goal frame with respect to EEF
/// \param offset Offset in z axis of the EEF
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::CreateGoalFrame(const double offset)
{
  // Create goal frame
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.frame_id = "panda_link8";
  transformStamped.child_frame_id = "goal";
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.transform.translation.z = offset;        // move only in z axis of the EEF
  transformStamped.transform.rotation.w = 1.0;              // keep fixed orientation
  // Send frame to tf
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformStamped);
}


///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::GetCurrentState Return a current state of the robot.
/// \return Returns current position of the robot EEF
///////////////////////////////////////////////////////////////////////
geometry_msgs::Pose FrankaJoystickControl::GetCurrentState()
{
  geometry_msgs::Pose current_pose = _group.getCurrentPose().pose;
  return current_pose;
}


///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::SendCartesianVelocityCommand Send a goal cartesian velocity command to the robot controller.
/// \param cartesian_velocity_command Command representing cartesian velocity [x, y, z, r, p, y]
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::SendCartesianVelocityCommand(const geometry_msgs::Twist cartesian_velocity_command)
{
  _cartesian_goal_publisher.publish(cartesian_velocity_command);
}

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::ErrorRecovery Sends error recovery message for Panda.
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::ErrorRecovery()
{
  franka_msgs::ErrorRecoveryActionGoal error_recovery_msg;
  _franka_error_recovery_publisher.publish(error_recovery_msg);
}

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::ChangeController
/// \param start_controller The name of the controller that should be started.
/// \param stop_controller  The name of the controller that should be stopped.
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::ChangeController(const std::string start_controller,
                                             const std::string stop_controller)
{
  controller_manager_msgs::SwitchController switch_controller_srv;
  switch_controller_srv.request.start_controllers.push_back(start_controller);
  switch_controller_srv.request.stop_controllers.push_back(stop_controller);
  switch_controller_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  if(!_switch_controller_service.call(switch_controller_srv))
  {
    ROS_ERROR("Failed to switch controllers (from  %s to %s)", start_controller.c_str(), stop_controller.c_str());
  }
  // Flag variable for collaborative mode
  if (start_controller == "force_controller")
  {
    collaborative_mode_flag = true;
  }
  else
  {
    collaborative_mode_flag = false;
  }
}

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::ButtonClickFunction Click function for buttons
/// \param BUTTON signal from button
/// \param button_flag Flag of button being pressed (not released)
///////////////////////////////////////////////////////////////////////
int FrankaJoystickControl::ButtonClickFunction(const bool &BUTTON, bool &button_flag)
{
  if (!BUTTON && button_flag)
  {
    button_flag = false;
    return 1;
  }
  else if(BUTTON)
  {
    button_flag = true;
  }
  return 0;
}