#include "franka_joystick_control/franka_joystick_control_lib.h"
#include <functional>

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::FrankaJoystickControl Initialization of moveit and ros communication features
/// \param nh ros::NodeHandle pointer
///////////////////////////////////////////////////////////////////////
FrankaJoystickControl::FrankaJoystickControl(ros::NodeHandle *nh)
{
  _cartesian_goal_publisher = nh->advertise<geometry_msgs::Twist>("/joystick_cartesian_goal", 1);
  _franka_error_recovery_publisher = nh->advertise<franka_msgs::ErrorRecoveryActionGoal>("/franka_control/error_recovery/goal", 1);
  _switch_controller_service = nh->serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
}



///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::joystickCallback
/// \param msg Contains signals from the joystick
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::joystickCallback(const joystick_msgs::Joystick::ConstPtr &msg)
{
  joystick_cmd.EEF_x_cmd  = (-msg->axis_2/32767);
  joystick_cmd.error_recovery = msg->button_3;

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


///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::FrankaStateCallback Callback function of FrankaStateController that computes FK after receiving new joints
/// \param msg Joint state message from publisher "/franka_state_controller/joint_states"
///////////////////////////////////////////////////////////////////////
void FrankaJoystickControl::FrankaStateCallback(const sensor_msgs::JointState &msg)
{
  auto T1 = DH_matrix(0,          0.333,      0,            msg.position[0]);
  auto T2 = DH_matrix(0,          0,          -M_PI_2,      msg.position[1]);
  auto T3 = DH_matrix(0,          0.316,      M_PI_2,       msg.position[2]);
  auto T4 = DH_matrix(0.0825,     0,          M_PI_2,       msg.position[3]);
  auto T5 = DH_matrix(-0.0825,    0.384,      -M_PI_2,      msg.position[4]);
  auto T6 = DH_matrix(0,          0,          M_PI_2,       msg.position[5]);
  auto T7 = DH_matrix(0.088,      0,          M_PI_2,       msg.position[6]);
  auto T8 = DH_matrix(0,          0.107,      0,            0);
  X_base_eef = T1*T2*T3*T4*T5*T6*T7*T8;
}

///////////////////////////////////////////////////////////////////////
/// \brief FrankaJoystickControl::DH_matrix Function for computing transformation matrix according to DH
/// \param a Distance between z_{i-1} and z_{i}
/// \param d Distance between x_{i-1} and x_{i}
/// \param alpha Angle between z_{i-1} and z_{i}
/// \param theta Joint angle
///////////////////////////////////////////////////////////////////////
Eigen::Matrix4d FrankaJoystickControl::DH_matrix(const double a, const double d, const double alpha, const double theta)
{
  Eigen::MatrixXd T(4,4); 
  double sin_alpha = sin(alpha), cos_alpha = cos(alpha);
  double sin_theta = sin(theta), cos_theta = cos(theta);
  T << cos_theta,              -sin_theta,                   0,                a,
       sin_theta*cos_alpha,    cos_theta*cos_alpha,          -sin_alpha,       -sin_alpha*d,
       sin_theta*sin_alpha,    cos_theta*sin_alpha,          cos_alpha,        cos_alpha*d,
       0,                      0,                            0,                1;
  return T;
}

