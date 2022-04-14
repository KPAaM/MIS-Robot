#include "ros/ros.h"
#include "franka_joystick_control/joystick_lib.h"
#include <joystick_msgs/Joystick.h>

//-------------------------------------------
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//TODO: In case you want to add another buttons
//      or axes you need to bind them in while loop
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//-------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_publisher");
  ros::NodeHandle nh;

  ros::Publisher joystick_pub = nh.advertise<joystick_msgs::Joystick>("joystick_feedback", 1);

  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");
  // Ensure that it was found so we can use it
  if (!joystick.isFound())
  {
    ROS_ERROR("Can't find joystick! Shutting down...");
  }

  // ------- MAIN LOOP ----------
  ros::Rate loop_rate(500);     // 500Hz
  joystick_msgs::Joystick msg;
  while (ros::ok())
  {
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      // Axes
      if (event.isAxis())
      {
        // Z axes
        if (event.number == 2)
        {
          msg.axis_2 = event.value;
        }
        // Y axes
        if (event.number == 0)
        {
          msg.axis_0 = event.value;
        }
        // X axes
        if (event.number == 1)
        {
          msg.axis_1 = event.value;
        }
      }
      // Buttons
      if (event.isButton())
      {
        //TODO: free button_0
        if (event.number == 0)
        {
          msg.button_0 = event.value;
        }
        // Colaborative mode
        if (event.number == 1)
        {
          msg.button_1 = event.value;
        }
        //TODO: free button_2
        if (event.number == 2)
        {
          msg.button_2 = event.value;
        }
        // Error recovery button
        if (event.number == 3)
        {
          msg.button_3 = event.value;
        }
      }
    }
    joystick_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
