/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/SelectionObj.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/ApplyWrenchDialogPrivate.hh"
#include "gazebo/gui/ApplyWrenchDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ApplyWrenchDialog::ApplyWrenchDialog(QWidget *_parent)
  : QDialog(_parent), dataPtr(new ApplyWrenchDialogPrivate)
{
  this->setObjectName("ApplyWrenchDialog");

  this->setWindowTitle(tr("Apply Force and Torque"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);
  this->setWindowModality(Qt::NonModal);

  this->dataPtr->modelLabel = new QLabel();

  // Links list
  QHBoxLayout *linkLayout = new QHBoxLayout();
  QLabel *linkLabel = new QLabel();
  linkLabel->setText(tr("Apply to link:"));
  this->dataPtr->linksComboBox = new QComboBox();

  linkLayout->addWidget(linkLabel);
  linkLayout->addWidget(this->dataPtr->linksComboBox);

  // Force position
  QCheckBox *forcePosCollapser = new QCheckBox();
  forcePosCollapser->setChecked(false);
  forcePosCollapser->setText("Reference Point");
  forcePosCollapser->setStyleSheet(
     "QCheckBox {\
        color: #d0d0d0;\
      }\
      QCheckBox::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QCheckBox::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(forcePosCollapser, SIGNAL(toggled(bool)), this, SLOT(ToggleForcePos(bool)));

  // CoM
  QLabel *comLabel = new QLabel();
  comLabel->setText(tr("Center of mass"));
  this->dataPtr->comCheckBox = new QCheckBox();
  this->dataPtr->comCheckBox->setChecked(true);
  connect(this->dataPtr->comCheckBox, SIGNAL(toggled(bool)), this,
      SLOT(ToggleComCheckBox(bool)));

  // Force Position X
  QLabel *forcePosXLabel = new QLabel();
  forcePosXLabel->setText(tr("X:"));
  QLabel *forcePosXUnitLabel = new QLabel();
  forcePosXUnitLabel->setText(tr("m"));

  this->dataPtr->forcePosXSpin = new QDoubleSpinBox();
  this->dataPtr->forcePosXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forcePosXSpin->setSingleStep(0.1);
  this->dataPtr->forcePosXSpin->setDecimals(3);
  this->dataPtr->forcePosXSpin->setValue(0);
  this->dataPtr->forcePosXSpin->setMaximumWidth(100);
  this->dataPtr->forcePosXSpin->installEventFilter(this);
  connect(this->dataPtr->forcePosXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForcePosXChanged(double)));

  // Force Position Y
  QLabel *forcePosYLabel = new QLabel();
  forcePosYLabel->setText(tr("Y:"));
  QLabel *forcePosYUnitLabel = new QLabel();
  forcePosYUnitLabel->setText(tr("m"));

  this->dataPtr->forcePosYSpin = new QDoubleSpinBox();
  this->dataPtr->forcePosYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forcePosYSpin->setSingleStep(0.1);
  this->dataPtr->forcePosYSpin->setDecimals(3);
  this->dataPtr->forcePosYSpin->setValue(0);
  this->dataPtr->forcePosYSpin->setMaximumWidth(100);
  this->dataPtr->forcePosYSpin->installEventFilter(this);
  connect(this->dataPtr->forcePosYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForcePosYChanged(double)));

  // Force Position Z
  QLabel *forcePosZLabel = new QLabel();
  forcePosZLabel->setText(tr("Z:"));
  QLabel *forcePosZUnitLabel = new QLabel();
  forcePosZUnitLabel->setText(tr("m"));

  this->dataPtr->forcePosZSpin = new QDoubleSpinBox();
  this->dataPtr->forcePosZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forcePosZSpin->setSingleStep(0.1);
  this->dataPtr->forcePosZSpin->setDecimals(3);
  this->dataPtr->forcePosZSpin->setValue(0);
  this->dataPtr->forcePosZSpin->setMaximumWidth(100);
  this->dataPtr->forcePosZSpin->installEventFilter(this);
  connect(this->dataPtr->forcePosZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForcePosZChanged(double)));

  QGridLayout *forcePosCollapsible = new QGridLayout();
  forcePosCollapsible->addWidget(comLabel, 0, 0);
  forcePosCollapsible->addWidget(this->dataPtr->comCheckBox, 0, 1);
  forcePosCollapsible->addWidget(forcePosXLabel, 1, 0);
  forcePosCollapsible->addWidget(this->dataPtr->forcePosXSpin, 1, 1);
  forcePosCollapsible->addWidget(forcePosXUnitLabel, 1, 2);
  forcePosCollapsible->addWidget(forcePosYLabel, 2, 0);
  forcePosCollapsible->addWidget(this->dataPtr->forcePosYSpin, 2, 1);
  forcePosCollapsible->addWidget(forcePosYUnitLabel, 2, 2);
  forcePosCollapsible->addWidget(forcePosZLabel, 3, 0);
  forcePosCollapsible->addWidget(this->dataPtr->forcePosZSpin, 3, 1);
  forcePosCollapsible->addWidget(forcePosZUnitLabel, 3, 2);

  this->dataPtr->forcePosCollapsibleWidget = new QWidget();
  this->dataPtr->forcePosCollapsibleWidget->setLayout(forcePosCollapsible);
  this->dataPtr->forcePosCollapsibleWidget->hide();

  QVBoxLayout *forcePosLayout = new QVBoxLayout();
  forcePosLayout->addWidget(forcePosCollapser);
  forcePosLayout->addWidget(this->dataPtr->forcePosCollapsibleWidget);

  // Force
  QCheckBox *forceCollapser = new QCheckBox();
  forceCollapser->setChecked(true);
  forceCollapser->setText("Force");
  forceCollapser->setStyleSheet(
     "QCheckBox {\
        color: #d0d0d0;\
      }\
      QCheckBox::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QCheckBox::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(forceCollapser, SIGNAL(toggled(bool)), this, SLOT(ToggleForce(bool)));

  // Force magnitude
  QLabel *forceMagLabel = new QLabel();
  forceMagLabel->setText(tr("Total:"));
  QLabel *forceMagUnitLabel = new QLabel();
  forceMagUnitLabel->setText(tr("N"));

  this->dataPtr->forceMagSpin = new QDoubleSpinBox();
  this->dataPtr->forceMagSpin->setRange(0, GZ_DBL_MAX);
  this->dataPtr->forceMagSpin->setSingleStep(0.1);
  this->dataPtr->forceMagSpin->setDecimals(3);
  this->dataPtr->forceMagSpin->setValue(1000);
  this->dataPtr->forceMagSpin->setMaximumWidth(100);
  this->dataPtr->forceMagSpin->installEventFilter(this);
  connect(this->dataPtr->forceMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceMagChanged(double)));

  // Force X
  QLabel *forceXLabel = new QLabel();
  forceXLabel->setText(tr("X:"));
  QLabel *forceXUnitLabel = new QLabel();
  forceXUnitLabel->setText(tr("N"));

  this->dataPtr->forceXSpin = new QDoubleSpinBox();
  this->dataPtr->forceXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forceXSpin->setSingleStep(0.1);
  this->dataPtr->forceXSpin->setDecimals(3);
  this->dataPtr->forceXSpin->setValue(1);
  this->dataPtr->forceXSpin->setMaximumWidth(100);
  this->dataPtr->forceXSpin->installEventFilter(this);
  connect(this->dataPtr->forceXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceXChanged(double)));

  // Force Y
  QLabel *forceYLabel = new QLabel();
  forceYLabel->setText(tr("Y:"));
  QLabel *forceYUnitLabel = new QLabel();
  forceYUnitLabel->setText(tr("N"));

  this->dataPtr->forceYSpin = new QDoubleSpinBox();
  this->dataPtr->forceYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forceYSpin->setSingleStep(0.1);
  this->dataPtr->forceYSpin->setDecimals(3);
  this->dataPtr->forceYSpin->setValue(0);
  this->dataPtr->forceYSpin->setMaximumWidth(100);
  this->dataPtr->forceYSpin->installEventFilter(this);
  connect(this->dataPtr->forceYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceYChanged(double)));

  // Force Z
  QLabel *forceZLabel = new QLabel();
  forceZLabel->setText(tr("Z:"));
  QLabel *forceZUnitLabel = new QLabel();
  forceZUnitLabel->setText(tr("N"));

  this->dataPtr->forceZSpin = new QDoubleSpinBox();
  this->dataPtr->forceZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->forceZSpin->setSingleStep(0.1);
  this->dataPtr->forceZSpin->setDecimals(3);
  this->dataPtr->forceZSpin->setValue(0);
  this->dataPtr->forceZSpin->setMaximumWidth(100);
  this->dataPtr->forceZSpin->installEventFilter(this);
  connect(this->dataPtr->forceZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceZChanged(double)));

  // Force clear
  QPushButton *forceClearButton = new QPushButton(tr("Clear"));
  connect(forceClearButton, SIGNAL(clicked()), this, SLOT(OnForceClear()));

  QGridLayout *forceVectorLayout = new QGridLayout();
  forceVectorLayout->addWidget(forceXLabel, 0, 1);
  forceVectorLayout->addWidget(this->dataPtr->forceXSpin, 0, 2);
  forceVectorLayout->addWidget(forceXUnitLabel, 0, 3);
  forceVectorLayout->addWidget(forceYLabel, 1, 1);
  forceVectorLayout->addWidget(this->dataPtr->forceYSpin, 1, 2);
  forceVectorLayout->addWidget(forceYUnitLabel, 1, 3);
  forceVectorLayout->addWidget(forceZLabel, 2, 1);
  forceVectorLayout->addWidget(this->dataPtr->forceZSpin, 2, 2);
  forceVectorLayout->addWidget(forceZUnitLabel, 2, 3);
  forceVectorLayout->addWidget(forceMagLabel, 3, 1);
  forceVectorLayout->addWidget(this->dataPtr->forceMagSpin, 3, 2);
  forceVectorLayout->addWidget(forceMagUnitLabel, 3, 3);
  forceVectorLayout->addWidget(forceClearButton, 4, 0);

  QVBoxLayout *forceCollapsible = new QVBoxLayout();
  forceCollapsible->addLayout(forcePosLayout);
  forceCollapsible->addLayout(forceVectorLayout);

  this->dataPtr->forceCollapsibleWidget = new QWidget();
  this->dataPtr->forceCollapsibleWidget->setLayout(forceCollapsible);
  this->dataPtr->forceCollapsibleWidget->show();

  QVBoxLayout *forceLayout = new QVBoxLayout();
  forceLayout->addWidget(forceCollapser);
  forceLayout->addWidget(this->dataPtr->forceCollapsibleWidget);

  // Torque
  QCheckBox *torqueCollapser = new QCheckBox();
  torqueCollapser->setChecked(false);
  torqueCollapser->setText("Torque");
  torqueCollapser->setStyleSheet(
     "QCheckBox {\
        color: #d0d0d0;\
      }\
      QCheckBox::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QCheckBox::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(torqueCollapser, SIGNAL(toggled(bool)), this, SLOT(ToggleTorque(bool)));

  // Torque magnitude
  QLabel *torqueMagLabel = new QLabel();
  torqueMagLabel->setText(tr("Total:"));
  QLabel *torqueMagUnitLabel = new QLabel();
  torqueMagUnitLabel->setText(tr("Nm"));

  this->dataPtr->torqueMagSpin = new QDoubleSpinBox();
  this->dataPtr->torqueMagSpin->setRange(0, GZ_DBL_MAX);
  this->dataPtr->torqueMagSpin->setSingleStep(0.1);
  this->dataPtr->torqueMagSpin->setDecimals(3);
  this->dataPtr->torqueMagSpin->setValue(0);
  this->dataPtr->torqueMagSpin->setMaximumWidth(100);
  this->dataPtr->torqueMagSpin->installEventFilter(this);
  connect(this->dataPtr->torqueMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueMagChanged(double)));

  // Torque X
  QLabel *torqueXLabel = new QLabel();
  torqueXLabel->setText(tr("X:"));
  QLabel *torqueXUnitLabel = new QLabel();
  torqueXUnitLabel->setText(tr("Nm"));

  this->dataPtr->torqueXSpin = new QDoubleSpinBox();
  this->dataPtr->torqueXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->torqueXSpin->setSingleStep(0.1);
  this->dataPtr->torqueXSpin->setDecimals(3);
  this->dataPtr->torqueXSpin->setValue(0);
  this->dataPtr->torqueXSpin->setMaximumWidth(100);
  this->dataPtr->torqueXSpin->installEventFilter(this);
  connect(this->dataPtr->torqueXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueXChanged(double)));

  // Torque Y
  QLabel *torqueYLabel = new QLabel();
  torqueYLabel->setText(tr("Y:"));
  QLabel *torqueYUnitLabel = new QLabel();
  torqueYUnitLabel->setText(tr("Nm"));

  this->dataPtr->torqueYSpin = new QDoubleSpinBox();
  this->dataPtr->torqueYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->torqueYSpin->setSingleStep(0.1);
  this->dataPtr->torqueYSpin->setDecimals(3);
  this->dataPtr->torqueYSpin->setValue(0);
  this->dataPtr->torqueYSpin->setMaximumWidth(100);
  this->dataPtr->torqueYSpin->installEventFilter(this);
  connect(this->dataPtr->torqueYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueYChanged(double)));

  // Torque Z
  QLabel *torqueZLabel = new QLabel();
  torqueZLabel->setText(tr("Z:"));
  QLabel *torqueZUnitLabel = new QLabel();
  torqueZUnitLabel->setText(tr("Nm"));

  this->dataPtr->torqueZSpin = new QDoubleSpinBox();
  this->dataPtr->torqueZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->torqueZSpin->setSingleStep(0.1);
  this->dataPtr->torqueZSpin->setDecimals(3);
  this->dataPtr->torqueZSpin->setValue(0);
  this->dataPtr->torqueZSpin->setMaximumWidth(100);
  this->dataPtr->torqueZSpin->installEventFilter(this);
  connect(this->dataPtr->torqueZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueZChanged(double)));

  // Torque clear
  QPushButton *torqueClearButton = new QPushButton(tr("Clear"));
  connect(torqueClearButton, SIGNAL(clicked()), this, SLOT(OnTorqueClear()));

  QGridLayout *torqueCollapsible = new QGridLayout();
  torqueCollapsible->addWidget(torqueXLabel, 0, 1);
  torqueCollapsible->addWidget(this->dataPtr->torqueXSpin, 0, 2);
  torqueCollapsible->addWidget(torqueXUnitLabel, 0, 3);
  torqueCollapsible->addWidget(torqueYLabel, 1, 1);
  torqueCollapsible->addWidget(this->dataPtr->torqueYSpin, 1, 2);
  torqueCollapsible->addWidget(torqueYUnitLabel, 1, 3);
  torqueCollapsible->addWidget(torqueZLabel, 2, 1);
  torqueCollapsible->addWidget(this->dataPtr->torqueZSpin, 2, 2);
  torqueCollapsible->addWidget(torqueZUnitLabel, 2, 3);
  torqueCollapsible->addWidget(torqueMagLabel, 3, 1);
  torqueCollapsible->addWidget(this->dataPtr->torqueMagSpin, 3, 2);
  torqueCollapsible->addWidget(torqueMagUnitLabel, 3, 3);
  torqueCollapsible->addWidget(torqueClearButton, 4, 0);

  this->dataPtr->torqueCollapsibleWidget = new QWidget();
  this->dataPtr->torqueCollapsibleWidget->setLayout(torqueCollapsible);
  this->dataPtr->torqueCollapsibleWidget->hide();

  QVBoxLayout *torqueLayout = new QVBoxLayout();
  torqueLayout->addWidget(torqueCollapser);
  torqueLayout->addWidget(this->dataPtr->torqueCollapsibleWidget);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *applyButton = new QPushButton("Apply");
  applyButton->setDefault(true);
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  mainLayout->addWidget(this->dataPtr->modelLabel);
  mainLayout->addLayout(linkLayout);
  mainLayout->addLayout(forceLayout);
  mainLayout->addLayout(torqueLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->dataPtr->requestMsg = NULL;
  this->dataPtr->requestPub.reset();
  this->dataPtr->responseSub.reset();

  this->dataPtr->mode = "force";
  this->dataPtr->comVector = math::Vector3::Zero;
  this->dataPtr->forceVector = 1000 * math::Vector3::UnitX;
  this->dataPtr->torqueVector = math::Vector3::Zero;

  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

  this->dataPtr->connections.push_back(
      event::Events::ConnectPreRender(
      boost::bind(&ApplyWrenchDialog::OnPreRender, this)));
}

/////////////////////////////////////////////////
ApplyWrenchDialog::~ApplyWrenchDialog()
{
  this->dataPtr->node->Fini();
  this->dataPtr->connections.clear();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetModel(std::string _modelName)
{
  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(_modelName);

  if (!vis)
  {
    gzerr << "Model [" << _modelName << "] could not be found." << std::endl;
    this->reject();
    return;
  }

  this->dataPtr->modelLabel->setText(
      ("Model: " + _modelName).c_str());

  this->dataPtr->linksComboBox->clear();

  for (unsigned int i = 0; i < vis->GetChildCount(); ++i)
  {
    rendering::VisualPtr childVis = vis->GetChild(i);

    uint32_t flags = childVis->GetVisibilityFlags();
    if (!((flags != GZ_VISIBILITY_ALL) && (flags & GZ_VISIBILITY_GUI)))
    {
      //this->dataPtr->linkList.push_back(childVis->GetName());

      std::string linkName = childVis->GetName();
      std::string unscopedLinkName = linkName.substr(linkName.find("::") + 2);
      this->dataPtr->linksComboBox->addItem(
          QString::fromStdString(unscopedLinkName));
    }
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetLink(std::string _linkName)
{
  // Request link message to get CoM
  this->dataPtr->requestPub = this->dataPtr->node->Advertise<msgs::Request>(
      "~/request");
  this->dataPtr->responseSub = this->dataPtr->node->Subscribe(
      "~/response", &ApplyWrenchDialog::OnResponse, this);

  this->dataPtr->requestMsg = msgs::CreateRequest("entity_info", _linkName);
  this->dataPtr->requestPub->Publish(*this->dataPtr->requestMsg);

  // Visual
  this->dataPtr->linkName = _linkName;
  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->dataPtr->linkName);

  this->dataPtr->linkVisual = vis;
  this->AttachVisuals();

  // Main window
  this->dataPtr->mainWindow = gui::get_main_window();
  this->dataPtr->mainWindow->installEventFilter(this);
}

///////////////////////////////////////////////////
void ApplyWrenchDialog::OnResponse(ConstResponsePtr &_msg)
{
  if (!this->dataPtr->requestMsg ||
      _msg->id() != this->dataPtr->requestMsg->id())
    return;

  if (_msg->has_type() && _msg->type() == "gazebo.msgs.Link")
  {
    this->dataPtr->linkMsg.ParseFromString(_msg->serialized_data());

    // Name
    if (!this->dataPtr->linkMsg.has_name())
      return;

    this->dataPtr->linkName = this->dataPtr->linkMsg.name();
    this->SetPublisher();

    // CoM
    if (this->dataPtr->linkMsg.has_inertial() &&
        this->dataPtr->linkMsg.inertial().has_pose())
    {
      this->SetCoM(msgs::Convert(
          this->dataPtr->linkMsg.inertial().pose()).pos);
      // Apply force at com by default
      this->SetForcePos(this->dataPtr->comVector);
    }
  }
  delete this->dataPtr->requestMsg;
  this->dataPtr->requestMsg = NULL;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApply()
{
  // publish wrench msg
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->dataPtr->forceVector);
  msgs::Set(msg.mutable_torque(), this->dataPtr->torqueVector);
  msgs::Set(msg.mutable_position(), this->dataPtr->forcePosVector);

  this->dataPtr->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnCancel()
{
  this->dataPtr->applyWrenchVisual->SetVisible(false);

  this->close();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForcePosXChanged(double /*_pX*/)
{
  this->NewForcePosVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForcePosYChanged(double /*_pY*/)
{
  this->NewForcePosVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForcePosZChanged(double /*_pZ*/)
{
  this->NewForcePosVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceMagChanged(double /*_magnitude*/)
{
  this->NewForceMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceXChanged(double /*_fX*/)
{
  this->NewForceVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceYChanged(double /*_fY*/)
{
  this->NewForceVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceZChanged(double /*_fZ*/)
{
  this->NewForceVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceClear()
{
  this->SetForce(math::Vector3::Zero);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueMagChanged(double /*_magnitude*/)
{
  this->NewTorqueMag();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueXChanged(double /*_fX*/)
{
  this->NewTorqueVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueYChanged(double /*_fY*/)
{
  this->NewTorqueVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueZChanged(double /*_fZ*/)
{
  this->NewTorqueVector();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueClear()
{
  this->SetTorque(math::Vector3::Zero);
}

//////////////////////////////////////////////////
void ApplyWrenchDialog::SetPublisher()
{
  std::string topicName = "~/";
  topicName += this->dataPtr->linkName + "/wrench";
  boost::replace_all(topicName, "::", "/");

  this->dataPtr->wrenchPub = this->dataPtr->node->Advertise<msgs::Wrench>(topicName);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::AttachVisuals()
{
  if (!this->dataPtr->applyWrenchVisual)
  {
    this->dataPtr->applyWrenchVisual.reset(new rendering::ApplyWrenchVisual(
        this->dataPtr->linkName + "__APPLY_WRENCH__", this->dataPtr->linkVisual));

    this->dataPtr->applyWrenchVisual->Load();
  }
  else
  {
    this->dataPtr->applyWrenchVisual->SetVisible(true);
  }

  this->SetTorque(this->dataPtr->torqueVector);
  this->SetForce(this->dataPtr->forceVector);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleForcePos(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->forcePosCollapsibleWidget->show();
  }
  else
  {
    this->dataPtr->forcePosCollapsibleWidget->hide();
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleComCheckBox(bool _checked)
{
  if (_checked)
  {
    this->SetForcePos(this->dataPtr->comVector);
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleForce(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->forceCollapsibleWidget->show();
  }
  else
  {
    this->dataPtr->forceCollapsibleWidget->hide();
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleTorque(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->torqueCollapsibleWidget->show();
  }
  else
  {
    this->dataPtr->torqueCollapsibleWidget->hide();
  }
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::OnMousePress(const common::MouseEvent & _event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  this->dataPtr->dragStart = math::Vector2i(0, 0);

  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos,
        this->dataPtr->manipState);

  if (vis)
    return false;

  // Register drag start point if on top of a handle
  if (this->dataPtr->manipState == "rot_z" ||
      this->dataPtr->manipState == "rot_y")
  {
    this->dataPtr->applyWrenchVisual->GetRotTool()->SetState(
        this->dataPtr->manipState);
    this->dataPtr->dragStart = _event.pressPos;

    math::Pose rotToolPose = this->dataPtr->applyWrenchVisual->GetRotTool()
        ->GetWorldPose();
    this->dataPtr->dragStartPose = rotToolPose;
  }
  return false;
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::OnMouseRelease(const common::MouseEvent & /*_event*/)
{
  return false;
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::OnMouseMove(const common::MouseEvent & _event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  // Dragging tool
  if (_event.dragging && _event.button == common::MouseEvent::LEFT &&
      this->dataPtr->dragStart.x != 0)
  {
    math::Vector3 normal;
    math::Vector3 axis;
    if (this->dataPtr->manipState == "rot_z")
    {
      normal = this->dataPtr->dragStartPose.rot.GetZAxis();
      axis = math::Vector3::UnitZ;
    }
    else if (this->dataPtr->manipState == "rot_y")
    {
      normal = this->dataPtr->dragStartPose.rot.GetYAxis();
      axis = math::Vector3::UnitY;
    }
    else
    {
      return false;
    }

    double offset = this->dataPtr->dragStartPose.pos.Dot(normal);

    math::Vector3 pressPoint;
    userCamera->GetWorldPointOnPlane(_event.pressPos.x, _event.pressPos.y,
          math::Plane(normal, offset), pressPoint);

    math::Vector3 newPoint;
    userCamera->GetWorldPointOnPlane(_event.pos.x, _event.pos.y,
        math::Plane(normal, offset), newPoint);

    math::Vector3 v1 = pressPoint - this->dataPtr->dragStartPose.pos;
    math::Vector3 v2 = newPoint - this->dataPtr->dragStartPose.pos;
    v1 = v1.Normalize();
    v2 = v2.Normalize();
    double signTest = v1.Cross(v2).Dot(normal);
    double angle = atan2((v1.Cross(v2)).GetLength(), v1.Dot(v2));

    if (signTest < 0)
      angle *= -1;

    if (_event.control)
      angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

    math::Quaternion rot(axis, angle);
    rot = this->dataPtr->dragStartPose.rot * rot;

    // Must rotate the tool here to make sure we have proper roll,
    // once the rotation gets transformed into a vector we lose a DOF
    this->dataPtr->applyWrenchVisual->GetRotTool()->SetWorldRotation(rot);

    math::Vector3 vec;
    math::Vector3 rotEuler;
    rotEuler = rot.GetAsEuler();
    vec.x = cos(rotEuler.z)*cos(rotEuler.y);
    vec.y = sin(rotEuler.z)*cos(rotEuler.y);
    vec.z = -sin(rotEuler.y);

    // To local frame
    vec = this->dataPtr->linkVisual->GetWorldPose().rot.RotateVectorReverse(vec);

    // Normalize new vector;
    if (vec == math::Vector3::Zero)
      vec = math::Vector3::UnitX;
    else
      vec.Normalize();

    if (this->dataPtr->mode == "force")
    {
      this->NewForceDirection(vec);
    }
    else if (this->dataPtr->mode == "torque")
    {
      this->NewTorqueDirection(vec);
    }
    return true;
  }
  // Highlight hovered tools
  else
  {
    userCamera->GetVisual(_event.pos, this->dataPtr->manipState);

    if (this->dataPtr->manipState == "rot_z" ||
      this->dataPtr->manipState == "rot_y")
    {
      this->dataPtr->applyWrenchVisual->GetRotTool()->SetState(
          this->dataPtr->manipState);
    }
    else
    {
      this->dataPtr->applyWrenchVisual->GetRotTool()->SetState("");
    }
  }

  return false;
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::eventFilter(QObject *_object, QEvent *_event)
{
  // Focus in spins
  if (_event->type() == QEvent::FocusIn)
  {
    if (_object == this->dataPtr->forceMagSpin ||
        _object == this->dataPtr->forceXSpin ||
        _object == this->dataPtr->forceYSpin ||
        _object == this->dataPtr->forceZSpin ||
        _object == this->dataPtr->forcePosXSpin ||
        _object == this->dataPtr->forcePosYSpin ||
        _object == this->dataPtr->forcePosZSpin)
    {
      this->SetForce(this->dataPtr->forceVector);
    }
    else if (_object == this->dataPtr->torqueMagSpin ||
             _object == this->dataPtr->torqueXSpin ||
             _object == this->dataPtr->torqueYSpin ||
             _object == this->dataPtr->torqueZSpin)
    {
      this->SetTorque(this->dataPtr->torqueVector);
    }
  }
  // Focus in main window
  else if (_event->type() == QEvent::ActivationChange)
  {
    if (!this->dataPtr->mainWindow)
      return false;

    if (_object == this->dataPtr->mainWindow)
    {
      if (!this->isActiveWindow() &&
          !this->dataPtr->mainWindow->isActiveWindow())
      {
        this->SetActive(false);
      }
    }
  }

  return false;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::changeEvent(QEvent *_event)
{
  // Focus in this dialog
  if (_event->type() == QEvent::ActivationChange)
  {
    if (!this->dataPtr->mainWindow)
      return;

    if (this->isActiveWindow() || this->dataPtr->mainWindow->isActiveWindow())
    {
      this->SetActive(true);
    }
    else
    {
      this->SetActive(false);
    }
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetSpinValue(QDoubleSpinBox *_spin, double _value)
{
  _spin->blockSignals(true);
  _spin->setValue(_value);
  _spin->blockSignals(false);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetWrenchMode(std::string _mode)
{
  // Update variable
  this->dataPtr->mode = _mode;
  // Not needed to set at visual
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetCoM(math::Vector3 _com)
{
  // Set com vector and send it to visuals
  this->dataPtr->comVector = _com;

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
    return;

  this->dataPtr->applyWrenchVisual->SetCoM(this->dataPtr->comVector);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetForcePos(math::Vector3 _forcePos)
{
  // Set forcePos vector and send it to visuals
  this->dataPtr->forcePosVector = _forcePos;

  // Spins
  this->SetSpinValue(this->dataPtr->forcePosXSpin, _forcePos.x);
  this->SetSpinValue(this->dataPtr->forcePosYSpin, _forcePos.y);
  this->SetSpinValue(this->dataPtr->forcePosZSpin, _forcePos.z);

  // Mode
  this->SetWrenchMode("force");

  // Check com box
  this->dataPtr->comCheckBox->setChecked((_forcePos == this->dataPtr->comVector));

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
    return;

  this->dataPtr->applyWrenchVisual->SetForcePos(this->dataPtr->forcePosVector);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewForcePosVector()
{
  // Update forcePos vector with values from XYZ spins
  this->SetForcePos(
      math::Vector3(this->dataPtr->forcePosXSpin->value(),
                    this->dataPtr->forcePosYSpin->value(),
                    this->dataPtr->forcePosZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetForce(math::Vector3 _force, bool _rotationByMouse)
{
  // Set force vector and send it to visuals and spins and set mode
  this->dataPtr->forceVector = _force;

  // Spins
  this->SetSpinValue(this->dataPtr->forceXSpin, _force.x);
  this->SetSpinValue(this->dataPtr->forceYSpin, _force.y);
  this->SetSpinValue(this->dataPtr->forceZSpin, _force.z);
  this->SetSpinValue(this->dataPtr->forceMagSpin, _force.GetLength());

  // Mode
  if (_force == math::Vector3::Zero)
  {
    if (this->dataPtr->torqueVector == math::Vector3::Zero)
      this->SetWrenchMode("none");
    else
      this->SetWrenchMode("torque");
  }
  else
  {
    this->SetWrenchMode("force");
  }

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
    return;

  this->dataPtr->applyWrenchVisual->SetForce(_force, _rotationByMouse);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewForceVector()
{
  // Update force vector with values from XYZ spins
  this->SetForce(math::Vector3(this->dataPtr->forceXSpin->value(),
                               this->dataPtr->forceYSpin->value(),
                               this->dataPtr->forceZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewForceMag()
{
  // Update force vector proportionally

  // Normalize current vector
  math::Vector3 v = this->dataPtr->forceVector;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  this->SetForce(v * this->dataPtr->forceMagSpin->value());
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewForceDirection(math::Vector3 _dir)
{
  // Update force vector with direction given by mouse, magnitude from spin

  // Normalize direction
  math::Vector3 v = _dir;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by magnitude
  this->SetForce(v * this->dataPtr->forceMagSpin->value(), true);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetTorque(math::Vector3 _torque, bool _rotationByMouse)
{
  // Set torque vector and send it to visuals and spins and set mode
  this->dataPtr->torqueVector = _torque;

  // Spins
  this->SetSpinValue(this->dataPtr->torqueXSpin, _torque.x);
  this->SetSpinValue(this->dataPtr->torqueYSpin, _torque.y);
  this->SetSpinValue(this->dataPtr->torqueZSpin, _torque.z);
  this->SetSpinValue(this->dataPtr->torqueMagSpin, _torque.GetLength());

  // Mode
  if (_torque == math::Vector3::Zero)
  {
    if (this->dataPtr->forceVector == math::Vector3::Zero)
      this->SetWrenchMode("none");
    else
      this->SetWrenchMode("force");
  }
  else
  {
    this->SetWrenchMode("torque");
  }

  // Visuals
  if (!this->dataPtr->applyWrenchVisual)
    return;

  this->dataPtr->applyWrenchVisual->SetTorque(_torque, _rotationByMouse);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewTorqueVector()
{
  // Update torque vector with values from XYZ spins
  this->SetTorque(math::Vector3(this->dataPtr->torqueXSpin->value(),
                               this->dataPtr->torqueYSpin->value(),
                               this->dataPtr->torqueZSpin->value()));
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewTorqueMag()
{
  // Update torque vector proportionally

  // Normalize current vector
  math::Vector3 v = this->dataPtr->torqueVector;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  this->SetTorque(v * this->dataPtr->torqueMagSpin->value());
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::NewTorqueDirection(math::Vector3 _dir)
{
  // Update torque vector with direction given by mouse, magnitude from spin

  // Normalize direction
  math::Vector3 v = _dir;
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by magnitude
  this->SetTorque(v * this->dataPtr->torqueMagSpin->value(), true);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetActive(bool _active)
{
  if (_active)
  {
    this->dataPtr->applyWrenchVisual->SetVisible(true);

    MouseEventHandler::Instance()->AddPressFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName,
        boost::bind(&ApplyWrenchDialog::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddReleaseFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName,
        boost::bind(&ApplyWrenchDialog::OnMouseRelease, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName,
        boost::bind(&ApplyWrenchDialog::OnMouseMove, this, _1));
  }
  else
  {
    this->dataPtr->applyWrenchVisual->SetVisible(false);

    MouseEventHandler::Instance()->RemovePressFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName);
    MouseEventHandler::Instance()->RemoveReleaseFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName);
    MouseEventHandler::Instance()->RemoveMoveFilter(
        "applyWrenchDialog_"+this->dataPtr->linkName);
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnPreRender()
{
  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->dataPtr->linkName);

  if (!vis)
    this->reject();
}
