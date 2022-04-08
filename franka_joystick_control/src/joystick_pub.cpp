#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "franka_joystick_control/joystick_lib.h"
#include "geometry_msgs/Pose.h"
#include <joystick_msgs/Joystick.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_publisher");
  ros::NodeHandle nh;

  ros::Publisher joystick_pub = nh.advertise<joystick_msgs::Joystick>("joystick_feedback", 1);

  // Create an instance of Joystick
  Joystick joystick("/dev/input/js0");
  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    ROS_ERROR("Can't find joystick! Shutting down...");
  }

  // Flag variables of buttons for the "click" function purpose
  bool button_1_press_flag = false;


  // ------- MAIN LOOP ----------
  ros::Rate loop_rate(100);     // 100Hz
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
      else if (event.isButton())
      {
        // Colaborative mode performed by click
        if (event.number == 1)
        {
          if (!event.value)
          {
            button_1_press_flag = true;
          }
          else if (event.value && button_1_press_flag) {
            // enable
            if (!msg.button_1)
            {
              msg.button_1 = true;
            }
            // disable
            else {
              msg.button_1= false;
            }
            button_1_press_flag = false;
          }
        }
        // Error recovery button
        if (event.number == 3)
        {
          msg.button_3 = event.value;
        }
      }
      else
      {
      }
    }
    joystick_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
