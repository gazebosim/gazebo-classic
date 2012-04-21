#include "gui/JointControlWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointControlWidget::JointControlWidget(const std::string &_modelName,
                                       QWidget *_parent)
  : QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(2, 2, 2, 2);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->node->Subscribe("~/response");
}

/////////////////////////////////////////////////
JointControlWidget::~JointControlWidget()
{
}

/////////////////////////////////////////////////
void JointControlWidget::Load(const std::string &/*_modelName*/)
{
}
