
#include <ros/ros.h>

#include <pr2_controllers_msgs/Pr2GripperCommand.h>

#include "velo_main_window.h"

VeloMainWindow::VeloMainWindow() : 
  root_nh_(""),
  ui_( new Ui::VeloMainWindow() ),
  next_command_close_(true)
{
  ui_->setupUi(this);
  connect( ui_->openCloseButton, SIGNAL( clicked() ), this, SLOT( buttonClicked() ) );
  pub_ = root_nh_.advertise<pr2_controllers_msgs::Pr2GripperCommand>("/l_gripper_controller/command", 10);
}

VeloMainWindow::~VeloMainWindow()
{
  delete ui_;
}

void VeloMainWindow::buttonClicked()
{
  double gap;
  double force;
  QString textChange;
  if (next_command_close_)
  {
    gap=0.135 * ((double)ui_->gapSlider->sliderPosition())/99.0;
    force = 10.0 * ((double)ui_->forceSlider->sliderPosition())/99.0;
    textChange = "Open";
  }
  else
  {
    gap=0.135;
    force = 5.0;
    textChange = "Close";
  }
  sendGripperCommand(gap, force);
  ui_->openCloseButton->setText(textChange);
  next_command_close_ = !next_command_close_;
}

void VeloMainWindow::sendGripperCommand(double gap, double force)
{
  printf("Going to gap %f with force %f\n", gap, force);
  pr2_controllers_msgs::Pr2GripperCommand command;
  command.position = gap;
  command.max_effort = force;
  pub_.publish(command);
}

