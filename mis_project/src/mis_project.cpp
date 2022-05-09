#include "mis_project.h"
#include "ui_mis_project.h"
#include <string>
#include <iostream>
#include <math.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QVariant>

#include <QtMultimedia/QCamera>
#include <QtMultimedia/QCameraInfo>
#include <QtMultimedia/QCameraImageCapture>
#include <QtMultimediaWidgets/QCameraViewfinder>
#include <QVBoxLayout>
#include <QAction>


#include <QMessageBox>
#include <QPalette>

#include <QtWidgets>


MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  nh_.reset(new ros::NodeHandle("~"));

  // camera
  /*
  mCamera = new QCamera(this);
  mCameraViewfinder = new QCameraViewfinder(this);
  mCameraImageCapture = new QCameraImageCapture(mCamera, this);
  mLayout = new QVBoxLayout;
  mCamera->setViewfinder(mCameraViewfinder);
  mLayout->addWidget(mCameraViewfinder);
  mLayout->setMargin(0);
  mAction = new QAction("Kamera",this);
  ui->widget_14->setLayout(mLayout);

  connect(mAction, &QAction::triggered, [&](){
    mCamera->start();
  });
*/


  //system("gnome-terminal -e cheese");

  // ================ RViz ==============
  QVBoxLayout* main_layout = new QVBoxLayout;
  render_panel_ = new rviz::RenderPanel();
  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(),manager_);
  ui->main_layout->addWidget(render_panel_);//  main_layout->addWidget(render_panel_);
  manager_->initialize();
  manager_->startUpdate();

  manager_->removeAllDisplays();
  manager_->setFixedFrame("panda_link0");
  manager_->createDisplay("rviz/Path","$(find xacro)/xacro $(find panda_moveit_config)/config/panda.srdf.xacro",true);
  rviz::Display *map = manager_->createDisplay( "rviz/Map", "adjustable map", true );
  map->subProp( "Topic" )->setValue( "/map" );

  rviz::Display *grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  map->subProp( "Topic" )->setValue( "/grid" );

  rviz::Display *robot = manager_->createDisplay( "rviz/RobotModel", "adjustable robot", "$(find xacro)/xacro $(find panda_moveit_config)/config/panda.srdf.xacro" );
  robot->subProp( "Robot Description" )->setValue( "robot_description" );//$(find xacro)/xacro $(find panda_moveit_config)/config/panda.srdf.xacro
  robot->subProp( "Reference Frame" )->setValue( "base_link" );

/*
  rviz::Display *laser = manager_->createDisplay( "rviz/LaserScan", "adjustable scan", true );
  laser->subProp( "Topic" )->setValue( "/scan" );
  laser->subProp( "Size (m)" )->setValue( "0.1" );
*/
  // ==========================================

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(100);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

  joystick_sub_ = nh.subscribe<joystick_msgs::Joystick>("joystick_feedback", 10, &MainWindow::joystickCallback, this);

}


MainWindow::~MainWindow()
{
  delete ui;
  delete ros_timer;
  delete manager_;
}

void MainWindow::spinOnce()
{
  if(ros::ok())
  {
    ros::spinOnce();
  }
  else
    QApplication::quit();
}

void MainWindow::joystickCallback(const joystick_msgs::Joystick::ConstPtr& joy)
{
  // ========================== Read Joystick States ==========================
  MainWindow::recoveryButton    = joy->button_3;
  MainWindow::colaborativeMode  = joy->button_1;
  MainWindow::joyHorizontal     = joy->axis_0;
  MainWindow::joyVertical       = joy->axis_1;
  MainWindow::joyRotation       = joy->axis_2;
  // ==========================================================================

  //ui->joyHorizontalLabel->setText(QString::number(MainWindow::joyHorizontal));
  //ui->joyVerticalLabel->setText(QString::number(MainWindow::joyVertical));


  // =================== Collaborative / Teleoperation Mode ====================
  if((MainWindow::collaborativeSwitch == false) && (MainWindow::colaborativeMode == true) && (MainWindow::colaborativeModeOld == false))
  {
    MainWindow::collaborativeSwitch = true;
    ui->label_18->setText(QString("Collaborative mode"));
  }
  if((MainWindow::collaborativeSwitch == true) && (MainWindow::colaborativeMode == false) && (MainWindow::colaborativeModeOld == false))
  {
    MainWindow::colaborativeModeOld = true;
    MainWindow::collaborativeSwitch = false;
  }
  if((MainWindow::collaborativeSwitch == false) && (MainWindow::colaborativeMode == true) && (MainWindow::colaborativeModeOld == true))
  {
    MainWindow::collaborativeSwitch = true;
    ui->label_18->setText(QString("Teleoperative mode"));
  }
  if((MainWindow::collaborativeSwitch == true) && (MainWindow::colaborativeMode == false) && (MainWindow::colaborativeModeOld == true))
  {
    MainWindow::colaborativeModeOld = false;
    MainWindow::collaborativeSwitch = false;
  }
  //ROS_INFO("Switch: %d, Mode: %d, Old: %d",MainWindow::collaborativeSwitch, MainWindow::colaborativeMode, MainWindow::colaborativeModeOld);
  // ==========================================================================

  MainWindow::areaRecognition();
  MainWindow::MotorControl(MainWindow::circleArea);
  if(MainWindow::circleArea != MainWindow::circleAreaOld)
  {
    MainWindow::MotorMotion();
    MainWindow::circleAreaOld = MainWindow::circleArea;
  }

}

void MainWindow::MotorControl(int circleArea)
{
  int low = 2, medium = 2, high = 3*servoCoefficient;
  // =================================== Set Servos Speed ===================================
  if( (MainWindow::joyHorizontal == 0) && (MainWindow::joyVertical == 0) )
  {
    MainWindow::servoVelocity[0]  = 0;
    MainWindow::servoVelocity[1]  = 0;
    MainWindow::servoVelocity[2]  = 0;
    ui->label_27->setText(QString("Stopped"));
    ui->label_28->setText(QString("Stopped"));
    ui->label_29->setText(QString("Stopped"));
    ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick0_ok.png);");
    ui->label_22->setText(QString::number(0,'f',2));
    if(MainWindow::zeroVelocity == false)
    {
      MainWindow::MotorMotion();
      MainWindow::zeroVelocity = true;
    }

  }else{
    MainWindow::zeroVelocity = false;
    switch(circleArea)
    {
    case 1:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*high;
      break;
    case 2:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*medium;
      break;
    case 3:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*medium;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 4:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*high;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 5:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*high;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 6:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*medium;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 7:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*medium;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 8:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*high;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 9:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*high;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 10:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*medium;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*low;
      break;
    case 11:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*medium;
      break;
    case 12:
      MainWindow::servoVelocity[0]  = MainWindow::servoDirection[0]*low;
      MainWindow::servoVelocity[1]  = MainWindow::servoDirection[1]*low;
      MainWindow::servoVelocity[2]  = MainWindow::servoDirection[2]*high;
      break;
    }
  }
  // ========================================================================================

}

void MainWindow::areaRecognition()
{
    // ======================================= Joystick Area Detection =======================================
    if( (MainWindow::joyHorizontal<0) && (MainWindow::joyVertical<0) )
    {
      MainWindow::area[0] = true;
      MainWindow::area[1] = false;
      MainWindow::area[2] = false;
      MainWindow::area[3] = false;
      MainWindow::angle   = 90 - ((atan2(-MainWindow::joyHorizontal, -MainWindow::joyVertical))*180/3.14);
      MainWindow::angleGui= (90-MainWindow::angle)+90;
    }
    if( (MainWindow::joyHorizontal<0) && (MainWindow::joyVertical>0) )
    {
      MainWindow::area[0] = false;
      MainWindow::area[1] = true;
      MainWindow::area[2] = false;
      MainWindow::area[3] = false;
      MainWindow::angle   = ((atan2(-MainWindow::joyHorizontal, -MainWindow::joyVertical))*180/3.14) - 90;
      MainWindow::angleGui= MainWindow::angle + 180;
    }
    if( (MainWindow::joyHorizontal>0) && (MainWindow::joyVertical>0) )
    {
      MainWindow::area[0] = false;
      MainWindow::area[1] = false;
      MainWindow::area[2] = true;
      MainWindow::area[3] = false;
      MainWindow::angle   = -90 - ((atan2(-MainWindow::joyHorizontal, -MainWindow::joyVertical))*180/3.14);
      MainWindow::angleGui= (90-MainWindow::angle) + 270;
    }
    if( (MainWindow::joyHorizontal>0) && (MainWindow::joyVertical<0) )
    {
      MainWindow::area[0] = false;
      MainWindow::area[1] = false;
      MainWindow::area[2] = false;
      MainWindow::area[3] = true;
      MainWindow::angle   = 90 + ((atan2(-MainWindow::joyHorizontal, -MainWindow::joyVertical))*180/3.14);
      MainWindow::angleGui= MainWindow::angle;
    }
    ui->label_22->setText(QString::number(MainWindow::angleGui,'f',2));
    // =======================================================================================================

    // ================================= Servo Direction and Circle Position =================================
    if( (MainWindow::area[0] == true ) && (MainWindow::angle<=90) && (MainWindow::angle>60) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] =  1;
      MainWindow::circleArea = 1;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick1_ok.png);");
    }
    if( (MainWindow::area[0] == true ) && (MainWindow::angle<=60) && (MainWindow::angle>30) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] =  1;
      MainWindow::circleArea = 2;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick2_ok.png);");
    }
    if( (MainWindow::area[0] == true ) && (MainWindow::angle<=30) && (MainWindow::angle>0) )
    {
      MainWindow::servoDirection[0] =  1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 3;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick3_ok.png);");
    }
    if( (MainWindow::area[1] == true ) && (MainWindow::angle<30) && (MainWindow::angle>=0) )
    {
      MainWindow::servoDirection[0] =  1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 4;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick4_ok.png);");
    }
    if( (MainWindow::area[1] == true ) && (MainWindow::angle<60) && (MainWindow::angle>=30) )
    {
      MainWindow::servoDirection[0] =  1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 5;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick5_ok.png);");
    }
    if( (MainWindow::area[1] == true ) && (MainWindow::angle<90) && (MainWindow::angle>=60) )
    {
      MainWindow::servoDirection[0] =  1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 6;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick6_ok.png);");
    }
    if( (MainWindow::area[2] == true ) && (MainWindow::angle<90) && (MainWindow::angle>=60) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] =  1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 7;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick7_ok.png);");
    }
    if( (MainWindow::area[2] == true ) && (MainWindow::angle<60) && (MainWindow::angle>=30) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] =  1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 8;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick8_ok.png);");
    }
    if( (MainWindow::area[2] == true ) && (MainWindow::angle<30) && (MainWindow::angle>=0) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] =  1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 9;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick9_ok.png);");
    }
    if( (MainWindow::area[3] == true ) && (MainWindow::angle<30) && (MainWindow::angle>=0) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] =  1;
      MainWindow::servoDirection[2] = -1;
      MainWindow::circleArea = 10;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick10_ok.png);");
    }
    if( (MainWindow::area[3] == true ) && (MainWindow::angle<60) && (MainWindow::angle>=30) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] =  1;
      MainWindow::circleArea = 11;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick11_ok.png);");
    }
    if( (MainWindow::area[3] == true ) && (MainWindow::angle<90) && (MainWindow::angle>=60) )
    {
      MainWindow::servoDirection[0] = -1;
      MainWindow::servoDirection[1] = -1;
      MainWindow::servoDirection[2] =  1;
      MainWindow::circleArea = 12;
      ui->label_13->setStyleSheet("background-image: url(/home/km/catkin_ws/src/mis_project/media/joystick12_ok.png);");
    }
    //ROS_INFO("Agnle: %f",MainWindow::angle);
    MainWindow::servoDirection[0] = 1*MainWindow::servoDirection[0];
    MainWindow::servoDirection[1] = -1*MainWindow::servoDirection[1];
    MainWindow::servoDirection[2] = -1*MainWindow::servoDirection[2];

    if(MainWindow::servoDirection[0]>0)
    {
      ui->label_27->setText(QString("Winding"));
    }else{
      ui->label_27->setText(QString("Unwinding"));
    }

    if(MainWindow::servoDirection[1]>0)
    {
      ui->label_28->setText(QString("Unwinding"));
    }else{
      ui->label_28->setText(QString("Winding"));
    }

    if(MainWindow::servoDirection[2]>0)
    {
      ui->label_29->setText(QString("Unwinding"));
    }else{
      ui->label_29->setText(QString("Winding"));
    }
    // =======================================================================================================
}

void MainWindow::MotorMotion()
{
  dynamixel_sdk_examples::SyncSetVelocity velocity;

  velocity.id1        = 1;
  velocity.id2        = 2;
  velocity.id3        = 3;
  velocity.velocity1  = MainWindow::servoVelocity[0];
  velocity.velocity2  = MainWindow::servoVelocity[1];
  velocity.velocity3  = MainWindow::servoVelocity[2];

  dynamixel_vel_pub_  = nh.advertise<dynamixel_sdk_examples::SyncSetVelocity>("/sync_set_velocity",1000);
  dynamixel_vel_pub_.publish(velocity);

}


void MainWindow::on_chobotSlider_actionTriggered(int action)
{
    //MainWindow::servoCoefficient =  ui->chobotSlider->value();
}

  /*
  ROS_INFO("cameras: %d",QCameraInfo::availableCameras().count());

  mLayout = new QVBoxLayout;
  const QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
  for (const QCameraInfo &cameraInfo : cameras) {
    qDebug() << "cameraInfo.deviceName: "<< cameraInfo.deviceName();
    mCamera = new QCamera(cameraInfo);
  }
  mCameraViewfinder = new QCameraViewfinder();
  mCameraViewfinder->show();
  mCamera->setViewfinder(mCameraViewfinder);
  //mLayout->addWidget(mCameraViewfinder);
  mCamera->start();
  //ui->frame_2->setLayout(mLayout);


  //mCamera = new QCamera("/dev/video0");

  //imageCapture = new QCameraImageCapture(mCamera);

  //ui->horizontalLayout_9->addWidget(mCameraViewfinder);
  //ui->label_10->setScaledContents(true);

  //ui->widget_14->setLayout(mLayout);
  //ui->frame_2->setLayout(mLayout);
  //ui->label_10->setScaledContents(true);






  //mCamera->setCaptureMode(QCamera::CaptureStillImage);



  qDebug() << "error: " <<mCamera->error() <<
                      "\n state:" << mCamera->state() <<
                      "\n status: " << mCamera->status() <<
                      "\n errorstring: " << mCamera->errorString() <<
                      "\n camptureMode: " << mCamera->captureMode() <<
                      "\n camera.lockStatus: " << mCamera->lockStatus() <<
                      "\n availableMetaData: " << mCamera->availableMetaData() <<
                      "\n camera.isAvailable: " << mCamera->isAvailable() <<
                      "\n viewfinder.isEnabled: "<< mCameraViewfinder->isEnabled() <<
                      "\n viewfinder.isVisible: " <<mCameraViewfinder->isVisible();

  /*
  imageCapture = new QCameraImageCapture(mCamera);
  mCamera->setCaptureMode(QCamera::CaptureStillImage);
  mCamera->start(); // Viewfinder frames start flowing
  //on half pressed shutter button
  mCamera->searchAndLock();
  //on shutter button pressed
  imageCapture->capture();
  //on shutter button released
  mCamera->unlock();
*/
  //mLayout->addWidget(mCameraViewfinder);
  //ui->widget_14->setLayout(mLayout);


  /*
  mCamera = new QCamera(this);
  mCameraViewfinder = new QCameraViewfinder(this);
  mCameraImageCapture = new QCameraImageCapture(mCamera, this);
  mLayout = new QVBoxLayout;
  mCamera->setViewfinder(mCameraViewfinder);
  mLayout->addWidget(mCameraViewfinder);
  mLayout->setMargin(0);
  mAction = new QAction("Kamera",this);
  ui->widget_14->setLayout(mLayout);

  connect(mAction, &QAction::triggered, [&](){
    mCamera->start();
  });
*/




void MainWindow::on_pushButton_2_clicked()
{
  system("gnome-terminal -e cheese");
}

