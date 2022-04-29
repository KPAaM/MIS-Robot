#include <franka_joystick_control/franka_joystick_control_lib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "franka_joystick_control");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  FrankaJoystickControl control(&nh);

  // Subscriber for joystick feedback
  ros::Subscriber sub = nh.subscribe("/joystick_feedback", 1, &FrankaJoystickControl::joystickCallback, &control);
  ros::Subscriber sub_franka_state = nh.subscribe("/franka_state_controller/joint_states", 1, &FrankaJoystickControl::FrankaStateCallback, &control);
  
  // Difference between goal_pose and current pose representing cartesian direction for velocity
  geometry_msgs::Twist delta_pose;  
  delta_pose.angular.x = 0.0;
  delta_pose.angular.y = 0.0;
  delta_pose.angular.z = 0.0;

  // Safety delay for getting first joint angles
  ros::Duration(1).sleep();
  ROS_INFO("Control interface is ready...");

  // Create goal transformation matrix
  const double OFFSET = 0.05;
  Eigen::MatrixXd  X_eef_goal(4,4);
  X_eef_goal << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, OFFSET,
                0, 0, 0, 1;
  Eigen::Matrix4d  X_base_goal = control.X_base_eef * X_eef_goal;

  //------Main loop--------
  ros::Rate rate(200.0);
  while(ros::ok())
  {
    // Recover from error
    if (control.joystick_cmd.error_recovery)
    {
      control.ErrorRecovery();
    }

    // ------ Changing controllers -------
    // Disable collaborative mode
    if (control.joystick_cmd.change_mode == control.TRAJECTORY_MODE
        && control.collaborative_mode_flag)
    {
      // Change controller
      ROS_INFO("Changing mode to joystick control...");
      control.ChangeController("cartesian_velocity_controller", "force_controller");
      ros::Duration(0.1).sleep();             // Safety delay for controller startup
      
    }
    // Enable collaborative mode
    else if (control.joystick_cmd.change_mode == control.FORCE_MODE
             && !control.collaborative_mode_flag) {
      ROS_INFO("Changing mode to collaborative...");
      control.ChangeController("force_controller", "cartesian_velocity_controller");
    }

    // Move according joystick model
    if(!control.collaborative_mode_flag)
     {
      // Compute direction for cartesian velocity controller
      X_base_goal = control.X_base_eef * X_eef_goal;
      delta_pose.linear.x = control.joystick_cmd.EEF_x_cmd * (X_base_goal(0,3) - control.X_base_eef(0,3));
      delta_pose.linear.y = control.joystick_cmd.EEF_x_cmd * (X_base_goal(1,3) - control.X_base_eef(1,3));
      delta_pose.linear.z = control.joystick_cmd.EEF_x_cmd * (X_base_goal(2,3) - control.X_base_eef(2,3));
            
      // Remove oscilations
      if (std::fabs(delta_pose.linear.x) < 0.005 ) delta_pose.linear.x = 0.0;
      if (std::fabs(delta_pose.linear.y) < 0.005 ) delta_pose.linear.y = 0.0;
      if (std::fabs(delta_pose.linear.z) < 0.005 ) delta_pose.linear.z = 0.0;
      
      // Send command
      control.SendCartesianVelocityCommand(delta_pose);
    }
    rate.sleep();
  }

  ros::waitForShutdown();
  return 0;
}
