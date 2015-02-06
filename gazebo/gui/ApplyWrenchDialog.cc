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

  this->dataPtr->messageLabel = new QLabel();
  this->dataPtr->messageLabel->setText(
      tr("Apply Force and Torque"));

  // Point
  QCheckBox *pointCollapser = new QCheckBox();
  pointCollapser->setChecked(false);
  pointCollapser->setText("Reference Point");
  pointCollapser->setStyleSheet(
     "QCheckBox {\
        color: #d0d0d0;\
      }\
      QCheckBox::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QCheckBox::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(pointCollapser, SIGNAL(toggled(bool)), this, SLOT(TogglePoint(bool)));

  // Point X
  QLabel *pointXLabel = new QLabel();
  pointXLabel->setText(tr("X:"));
  QLabel *pointXUnitLabel = new QLabel();
  pointXUnitLabel->setText(tr("m"));

  this->dataPtr->pointXSpin = new QDoubleSpinBox();
  this->dataPtr->pointXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->pointXSpin->setSingleStep(0.1);
  this->dataPtr->pointXSpin->setDecimals(3);
  this->dataPtr->pointXSpin->setValue(0);
  connect(this->dataPtr->pointXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointXChanged(double)));

  // Point Y
  QLabel *pointYLabel = new QLabel();
  pointYLabel->setText(tr("Y:"));
  QLabel *pointYUnitLabel = new QLabel();
  pointYUnitLabel->setText(tr("m"));

  this->dataPtr->pointYSpin = new QDoubleSpinBox();
  this->dataPtr->pointYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->pointYSpin->setSingleStep(0.1);
  this->dataPtr->pointYSpin->setDecimals(3);
  this->dataPtr->pointYSpin->setValue(0);
  connect(this->dataPtr->pointYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointYChanged(double)));

  // Point Z
  QLabel *pointZLabel = new QLabel();
  pointZLabel->setText(tr("Z:"));
  QLabel *pointZUnitLabel = new QLabel();
  pointZUnitLabel->setText(tr("m"));

  this->dataPtr->pointZSpin = new QDoubleSpinBox();
  this->dataPtr->pointZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->dataPtr->pointZSpin->setSingleStep(0.1);
  this->dataPtr->pointZSpin->setDecimals(3);
  this->dataPtr->pointZSpin->setValue(0);
  connect(this->dataPtr->pointZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointZChanged(double)));

  QGridLayout *pointCollapsible = new QGridLayout();
  pointCollapsible->addWidget(pointXLabel, 0, 0);
  pointCollapsible->addWidget(this->dataPtr->pointXSpin, 0, 1);
  pointCollapsible->addWidget(pointXUnitLabel, 0, 2);
  pointCollapsible->addWidget(pointYLabel, 1, 0);
  pointCollapsible->addWidget(this->dataPtr->pointYSpin, 1, 1);
  pointCollapsible->addWidget(pointYUnitLabel, 1, 2);
  pointCollapsible->addWidget(pointZLabel, 2, 0);
  pointCollapsible->addWidget(this->dataPtr->pointZSpin, 2, 1);
  pointCollapsible->addWidget(pointZUnitLabel, 2, 2);

  this->dataPtr->pointCollapsibleWidget = new QWidget();
  this->dataPtr->pointCollapsibleWidget->setLayout(pointCollapsible);
  this->dataPtr->pointCollapsibleWidget->hide();

  QVBoxLayout *pointLayout = new QVBoxLayout();
  pointLayout->addWidget(pointCollapser);
  pointLayout->addWidget(this->dataPtr->pointCollapsibleWidget);

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
  connect(this->dataPtr->forceZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceZChanged(double)));

  QGridLayout *forceCollapsible = new QGridLayout();
  forceCollapsible->addWidget(forceXLabel, 0, 0);
  forceCollapsible->addWidget(this->dataPtr->forceXSpin, 0, 1);
  forceCollapsible->addWidget(forceXUnitLabel, 0, 2);
  forceCollapsible->addWidget(forceYLabel, 1, 0);
  forceCollapsible->addWidget(this->dataPtr->forceYSpin, 1, 1);
  forceCollapsible->addWidget(forceYUnitLabel, 1, 2);
  forceCollapsible->addWidget(forceZLabel, 2, 0);
  forceCollapsible->addWidget(this->dataPtr->forceZSpin, 2, 1);
  forceCollapsible->addWidget(forceZUnitLabel, 2, 2);
  forceCollapsible->addWidget(forceMagLabel, 3, 0);
  forceCollapsible->addWidget(this->dataPtr->forceMagSpin, 3, 1);
  forceCollapsible->addWidget(forceMagUnitLabel, 3, 2);

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
  connect(this->dataPtr->torqueZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueZChanged(double)));

  QGridLayout *torqueCollapsible = new QGridLayout();
  torqueCollapsible->addWidget(torqueXLabel, 0, 0);
  torqueCollapsible->addWidget(this->dataPtr->torqueXSpin, 0, 1);
  torqueCollapsible->addWidget(torqueXUnitLabel, 0, 2);
  torqueCollapsible->addWidget(torqueYLabel, 1, 0);
  torqueCollapsible->addWidget(this->dataPtr->torqueYSpin, 1, 1);
  torqueCollapsible->addWidget(torqueYUnitLabel, 1, 2);
  torqueCollapsible->addWidget(torqueZLabel, 2, 0);
  torqueCollapsible->addWidget(this->dataPtr->torqueZSpin, 2, 1);
  torqueCollapsible->addWidget(torqueZUnitLabel, 2, 2);
  torqueCollapsible->addWidget(torqueMagLabel, 3, 0);
  torqueCollapsible->addWidget(this->dataPtr->torqueMagSpin, 3, 1);
  torqueCollapsible->addWidget(torqueMagUnitLabel, 3, 2);

  this->dataPtr->torqueCollapsibleWidget = new QWidget();
  this->dataPtr->torqueCollapsibleWidget->setLayout(torqueCollapsible);
  this->dataPtr->torqueCollapsibleWidget->hide();

  QVBoxLayout *torqueLayout = new QVBoxLayout();
  torqueLayout->addWidget(torqueCollapser);
  torqueLayout->addWidget(this->dataPtr->torqueCollapsibleWidget);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *applyButton = new QPushButton("&Apply");
  applyButton->setDefault(true);
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  mainLayout->addWidget(this->dataPtr->messageLabel);
//  mainLayout->addLayout(pointLayout);
  mainLayout->addLayout(forceLayout);
  mainLayout->addLayout(torqueLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  this->UpdateForceVector();
  this->UpdateTorqueVector();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);

  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

  MouseEventHandler::Instance()->AddPressFilter("applyWrenchDialog",
      boost::bind(&ApplyWrenchDialog::OnMousePress, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("applyWrenchDialog",
      boost::bind(&ApplyWrenchDialog::OnMouseRelease, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("applyWrenchDialog",
      boost::bind(&ApplyWrenchDialog::OnMouseMove, this, _1));
}

/////////////////////////////////////////////////
ApplyWrenchDialog::~ApplyWrenchDialog()
{
  this->dataPtr->node->Fini();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetLink(std::string _linkName)
{
  std::string msg = "Apply Force and Torque\n\nApply to " + _linkName + "\n";
  this->dataPtr->messageLabel->setText(msg.c_str());
  this->dataPtr->linkName = _linkName;
  this->SetPublisher();

  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
        GetVisual(this->dataPtr->linkName);

  this->dataPtr->linkVisual = vis;
  this->AttachVisuals();
}

///////////////////////////////////////////////////
void ApplyWrenchDialog::SetMode(rendering::ApplyWrenchVisual::WrenchModes _mode)
{
  this->dataPtr->wrenchMode = _mode;
  if (this->dataPtr->applyWrenchVisual)
    this->dataPtr->applyWrenchVisual->SetMode(this->dataPtr->wrenchMode);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApply()
{
  // publish wrench msg
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->dataPtr->forceVector);
  msgs::Set(msg.mutable_torque(), this->dataPtr->torqueVector);

  this->dataPtr->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnCancel()
{
  this->dataPtr->applyWrenchVisual->SetVisible(false);

  MouseEventHandler::Instance()->RemovePressFilter("applyWrenchDialog");
  MouseEventHandler::Instance()->RemoveReleaseFilter("applyWrenchDialog");
  MouseEventHandler::Instance()->RemoveMoveFilter("applyWrenchDialog");

  this->close();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnPointXChanged(double /*_pX*/)
{
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnPointYChanged(double /*_pY*/)
{
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnPointZChanged(double /*_pZ*/)
{
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceMagChanged(double /*_magnitude*/)
{
  this->UpdateForceVector();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceXChanged(double /*_fX*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceYChanged(double /*_fY*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceZChanged(double /*_fZ*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueMagChanged(double /*_magnitude*/)
{
  this->UpdateTorqueVector();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueXChanged(double /*_fX*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueYChanged(double /*_fY*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueZChanged(double /*_fZ*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
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
void ApplyWrenchDialog::CalculateForce()
{
  this->dataPtr->forceVector =
      math::Vector3(this->dataPtr->forceXSpin->value(),
                    this->dataPtr->forceYSpin->value(),
                    this->dataPtr->forceZSpin->value());

  this->UpdateForceVisual();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::CalculateTorque()
{
  this->dataPtr->torqueVector =
      math::Vector3(this->dataPtr->torqueXSpin->value(),
                    this->dataPtr->torqueYSpin->value(),
                    this->dataPtr->torqueZSpin->value());

  this->UpdateTorqueVisual();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVisual()
{
  if (!this->dataPtr->applyWrenchVisual)
    return;

  bool pleaseTryToRotateTheTool = true;
  if (this->dataPtr->updatingByMouse > 0)
  {
    pleaseTryToRotateTheTool = false;
    this->dataPtr->updatingByMouse--;
  }

  this->dataPtr->applyWrenchVisual->UpdateForce(this->dataPtr->forceVector,
      pleaseTryToRotateTheTool);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVisual()
{
  if (!this->dataPtr->applyWrenchVisual)
    return;

  bool pleaseTryToRotateTheTool = true;
  if (this->dataPtr->updatingByMouse > 0)
  {
    pleaseTryToRotateTheTool = false;
    this->dataPtr->updatingByMouse--;
  }

  this->dataPtr->applyWrenchVisual->UpdateTorque(this->dataPtr->torqueVector,
      pleaseTryToRotateTheTool);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceMag()
{
  this->dataPtr->forceMagSpin->setValue(sqrt(
      pow(this->dataPtr->forceXSpin->value(), 2) +
      pow(this->dataPtr->forceYSpin->value(), 2) +
      pow(this->dataPtr->forceZSpin->value(), 2)));
  this->CalculateForce();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->dataPtr->forceXSpin->value(),
                                  this->dataPtr->forceYSpin->value(),
                                  this->dataPtr->forceZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->dataPtr->forceMagSpin->value();

  this->UpdateForceVectorSpins(v);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVectorSpins(math::Vector3 _fV)
{
  this->dataPtr->forceXSpin->setValue(_fV.x);
  this->dataPtr->forceYSpin->setValue(_fV.y);
  this->dataPtr->forceZSpin->setValue(_fV.z);

  this->CalculateForce();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueMag()
{
  this->dataPtr->torqueMagSpin->setValue(sqrt(
      pow(this->dataPtr->torqueXSpin->value(), 2) +
      pow(this->dataPtr->torqueYSpin->value(), 2) +
      pow(this->dataPtr->torqueZSpin->value(), 2)));
  this->CalculateTorque();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->dataPtr->torqueXSpin->value(),
                                   this->dataPtr->torqueYSpin->value(),
                                   this->dataPtr->torqueZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->dataPtr->torqueMagSpin->value();

  this->UpdateTorqueVectorSpins(v);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVectorSpins(math::Vector3 _tV)
{
  this->dataPtr->torqueXSpin->setValue(_tV.x);
  this->dataPtr->torqueYSpin->setValue(_tV.y);
  this->dataPtr->torqueZSpin->setValue(_tV.z);

  this->CalculateTorque();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::AttachVisuals()
{
  if (!this->dataPtr->applyWrenchVisual)
  {
    this->dataPtr->applyWrenchVisual.reset(new rendering::ApplyWrenchVisual(this->dataPtr->linkName +
        "__APPLY_WRENCH__", this->dataPtr->linkVisual));

    this->dataPtr->applyWrenchVisual->Load();
    this->dataPtr->applyWrenchVisual->SetMode(this->dataPtr->wrenchMode);
  }
  else
  {
    this->dataPtr->applyWrenchVisual->SetVisible(true);
  }

  this->CalculateForce();
  this->CalculateTorque();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::TogglePoint(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->pointCollapsibleWidget->show();
  }
  else
  {
    this->dataPtr->pointCollapsibleWidget->hide();
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::ToggleForce(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->forceCollapsibleWidget->show();
    this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
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
    this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
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

    if (this->dataPtr->wrenchMode == rendering::ApplyWrenchVisual::FORCE)
    {
      // figure out why UpdateForceVisual gets called 5 times when moving the
      // mouse and fix this number
      this->dataPtr->updatingByMouse = 5;

      vec = vec * this->dataPtr->forceMagSpin->value();
      this->UpdateForceVectorSpins(vec);
    }
    else if (this->dataPtr->wrenchMode == rendering::ApplyWrenchVisual::TORQUE)
    {
      vec = vec * this->dataPtr->torqueMagSpin->value();
      this->UpdateTorqueVectorSpins(vec);
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
  if (_event->type() == QEvent::FocusIn)
  {
    if (_object == this->dataPtr->forceMagSpin ||
        _object == this->dataPtr->forceXSpin ||
        _object == this->dataPtr->forceYSpin ||
        _object == this->dataPtr->forceZSpin)
    {
        this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::FORCE);
    }
    else if (_object == this->dataPtr->torqueMagSpin ||
             _object == this->dataPtr->torqueXSpin ||
             _object == this->dataPtr->torqueYSpin ||
             _object == this->dataPtr->torqueZSpin)
    {
        this->SetMode(rendering::ApplyWrenchVisual::WrenchModes::TORQUE);
    }
  }
  return false;
}
