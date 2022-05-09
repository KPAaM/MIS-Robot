#ifndef MIS_PROJECT_H
#define MIS_PROJECT_H

#include <QMainWindow>
#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include "joystick_msgs/Joystick.h"
#include <geometry_msgs/Twist.h>
#include <dynamixel_sdk_examples/SyncSetPosition.h>
#include <dynamixel_sdk_examples/SyncSetVelocity.h>
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
  //void openLocal();
  //void openUrl();
  void on_chobotSlider_actionTriggered(int action);

  void on_pushButton_2_clicked();

private:
  Ui::MainWindow *ui;
  QTimer *ros_timer;

  char *command;

  bool area[4];
  bool zeroVelocity = false;
  bool recoveryButton = false;                                  // button_1
  bool colaborativeMode = false, colaborativeModeOld = false;   // button_3
  bool collaborativeSwitch = false;

  int joyHorizontal, joyVertical, joyRotation;
  int servoDirection[3];
  int servoVelocity[3];
  int servoCoefficient = 1;
  int circleArea;
  int circleAreaOld;
  float angle, angleGui;


  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;

  ros::NodeHandlePtr nh_;
  ros::NodeHandle nh;
  ros::Subscriber joystick_sub_;
  ros::Publisher joystick_pub_;
  ros::Publisher dynamixel_vel_pub_;
  ros::Publisher hello_pub_;

  // camera
  /*
  QCamera *mCamera;
  QCameraInfo *mCameraInfo;
  QCameraViewfinder *mCameraViewfinder;
  QGraphicsView *mGraphicsView;
  QCameraImageCapture *mCameraImageCapture;
  QVBoxLayout *mLayout;
  QAction *mAction;
  QCameraImageCapture *imageCapture;
  */
};

#endif // MIS_PROJECT_H
