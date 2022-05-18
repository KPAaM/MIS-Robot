#ifndef MIS_PROJECT_H
#define MIS_PROJECT_H

#include <QMainWindow>
#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include "joystick_msgs/Joystick.h"
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk_examples/SyncSetPosition.h>
#include <dynamixel_sdk_examples/SyncSetVelocity.h>
#include <dynamixel_sdk_examples/SetVelocity.h>
#include <dynamixel_sdk_examples/SetStop.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include <QtMultimedia/QCamera>
#include <QtMultimedia/QCameraInfo>
#include <QtMultimedia/QCameraImageCapture>
#include <QtMultimedia/QMediaRecorder>
#include <QtMultimediaWidgets/QCameraViewfinder>
#include <QScopedPointer>
#include <QVBoxLayout>
#include <QGraphicsView>
#include <franka_msgs/ErrorRecoveryActionGoal.h>

namespace Ui {
class MainWindow;
class Display;
class RenderPanel;
class VisualizationManager;
}



class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr); //nullptr
  ~MainWindow();
  void joystickCallback(const joystick_msgs::Joystick::ConstPtr& joy);
  void areaRecognition();
  void MotorControl(int circleArea);
  void MotorMotion();

public slots:
  void spinOnce();

private slots:
  void on_pushButton_12_clicked();
  void on_pushButton_11_clicked();
  void on_pushButton_10_clicked();
  void on_pushButton_9_clicked();
  void on_horizontalSlider_actionTriggered(int action);
  void on_pushButton_8_clicked();
  void on_pushButton_7_clicked();
  void on_pushButton_6_clicked();
  void on_pushButton_5_clicked();
  void on_pushButton_4_clicked();
  void on_pushButton_3_clicked();
  void on_horizontalSlider_2_actionTriggered(int action);
  void on_pushButton_clicked();
  void on_pushButton_2_clicked();

private:
  Ui::MainWindow *ui;
  QTimer *ros_timer;

  bool area[4];                                                 // 4 circle quadrants
  bool zeroVelocity = false;                                    // auxiliary variable
  bool recoveryButton = false;                                  // button_1
  bool colaborativeMode = false, colaborativeModeOld = false;   // button_3
  bool collaborativeSwitch = false;                             // auxiliary variable

  int joyHorizontal, joyVertical, joyRotation;                  // joystick motion
  int servoDirection[3];                                        // direction of individual motors
  int servoVelocity[3];                                         // velocity of individual motors
  int servoCoefficient = 1;                                     // coeficient for motor velocity value
  int circleArea;                                               // 12 areas of joystick motion
  int circleAreaOld;                                            // previous joystick area
  float angle, angleGui;                                        // angle of joystick
  std_msgs::Float64 robot_sensitivity;                          // velocity of Panda in z-axis


  rviz::VisualizationManager* manager_;                         // rviz
  rviz::RenderPanel* render_panel_;                             // rviz
  rviz::Display* grid_;                                         // rviz

  ros::NodeHandlePtr nh_;
  ros::NodeHandle nh;
  ros::Subscriber joystick_sub_;                                // joystick subscriber
  ros::Publisher joystick_pub_;                                 // joystick publisher
  ros::Publisher dynamixel_vel_pub_;                            // publisher for motor motion
  ros::Publisher dynamixel_vel_pub_zero_;                       // publisher for motion winding/unwinding stop
  ros::Publisher dynamixel_stop_;                               // publisher for releasing of all motors (free shaft of motor)
  ros::Publisher _franka_error_recovery_publisher;              // publisher for error recovery message
  ros::Publisher robot_sensitivity_;                            // publisher for change Panda velocity in z-axis
};

#endif // MIS_PROJECT_H
