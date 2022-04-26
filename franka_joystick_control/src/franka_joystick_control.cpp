#include <franka_joystick_control/franka_joystick_control_lib.h>

//BUG: robi pauzu medzi jednotlivymi framemami, mozno dat scaling pre konst rychlost

int main(int argc, char **argv)
{
  ros::init(argc, argv, "franka_joystick_control");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  FrankaJoystickControl control(&nh);
  ros::Duration(2).sleep();             // Safety delay for moveit to start up

  // Subscriber for joystick feedback
  ros::Subscriber sub = nh.subscribe("/joystick_feedback", 1, &FrankaJoystickControl::joystickCallback, &control);

  // tf2 for listening tf goal_position
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::Pose goal_pose = control.GetCurrentState();
  geometry_msgs::Pose current_pose = control.GetCurrentState();
  geometry_msgs::Twist delta_pose;  //difference between goal_pose and current pose representing cartesian velocity

  control.CreateGoalFrame(0.05);
  ros::Duration(0.1).sleep();             // Safety delay for moveit to start up
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("panda_link0", "goal",
                            ros::Time(0));
    goal_pose.position.x = transformStamped.transform.translation.x;
    goal_pose.position.y = transformStamped.transform.translation.y;
    goal_pose.position.z = transformStamped.transform.translation.z;

    // Compute direction for cartesian velocity controller
    delta_pose.linear.x = (goal_pose.position.x - current_pose.position.x);
    delta_pose.linear.y = (goal_pose.position.y - current_pose.position.y);
    delta_pose.linear.z = (goal_pose.position.z - current_pose.position.z);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("Can't get transformation...");
  }

  ROS_INFO("Control interface is ready...");

  //------Main loop--------
  ros::Rate rate(250.0);
  while(ros::ok())
  {
    // Recover from error
    if (control.joystick_cmd.error_recovery)
    {
      control.ErrorRecovery();
      control.joystick_cmd.error_recovery = false;
      ROS_INFO("Request for recovery from error send...");
    }

    // ------ Changing controllers -------
    // Disable collaborative mode
    if (control.joystick_cmd.change_mode == control.TRAJECTORY_MODE
        && control.collaborative_mode_flag)
    {
      ROS_INFO("Changing mode to joystick control...");
      control.ChangeController("cartesian_velocity_controller", "force_controller");
      control.CreateGoalFrame(0.05);
      ros::Duration(0.1).sleep();             // Safety delay for moveit to start up
      geometry_msgs::TransformStamped transformStamped;
      try {
        transformStamped = tfBuffer.lookupTransform("panda_link0", "goal",
                                ros::Time(0));
        goal_pose.position.x = transformStamped.transform.translation.x;
        goal_pose.position.y = transformStamped.transform.translation.y;
        goal_pose.position.z = transformStamped.transform.translation.z;

        // Compute direction for cartesian velocity controller
        delta_pose.linear.x = (goal_pose.position.x - current_pose.position.x);
        delta_pose.linear.y = (goal_pose.position.y - current_pose.position.y);
        delta_pose.linear.z = (goal_pose.position.z - current_pose.position.z);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("Can't get transformation...");
      }
    }
    // Enable collaborative mode
    else if (control.joystick_cmd.change_mode == control.FORCE_MODE
             && !control.collaborative_mode_flag) {
      ROS_INFO("Changing mode to collaborative...");
      control.ChangeController("force_controller", "cartesian_velocity_controller");
    }


    // Move according to a goal frame
    if(!control.collaborative_mode_flag)
    {
      // Compute direction for cartesian velocity controller
      delta_pose.linear.x *= control.joystick_cmd.EEF_x_cmd ;
      delta_pose.linear.y *= control.joystick_cmd.EEF_x_cmd ;
      delta_pose.linear.z *= control.joystick_cmd.EEF_x_cmd ;
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
