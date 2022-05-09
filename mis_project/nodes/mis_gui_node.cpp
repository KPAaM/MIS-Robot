#include <QApplication>
#include <QIcon>
#include "mis_project.h"


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "mis_gui_node");//,ros::init_options::AnonymousName

  QApplication a(argc, argv);
  MainWindow w;

  w.showMaximized();
  w.show();
  return a.exec();
}
