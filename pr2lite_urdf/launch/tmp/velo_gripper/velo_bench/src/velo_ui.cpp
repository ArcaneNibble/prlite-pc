#include <QApplication>

#include <ros/ros.h>

#include "velo_main_window.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "velo_ui");
  
  QApplication app(argc, argv);
  app.setOrganizationName("QtProject");
  app.setApplicationName("Application Example");
  VeloMainWindow window;
  window.show();
  return app.exec();
}
