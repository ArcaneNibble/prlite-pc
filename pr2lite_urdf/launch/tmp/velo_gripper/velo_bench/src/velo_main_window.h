
#include <QMainWindow>

#include <ros/ros.h>

namespace Ui
{
class VeloMainWindow;
}

#include "ui_velo_window.h"

class VeloMainWindow : public QMainWindow
{
Q_OBJECT
private:
  ros::NodeHandle root_nh_;
  ros::Publisher pub_;
  Ui::VeloMainWindow* ui_;
  bool next_command_close_;
public:
  VeloMainWindow();
  ~VeloMainWindow();
private:
  void sendGripperCommand(double gap, double force);
private Q_SLOTS:
  virtual void buttonClicked();
};

