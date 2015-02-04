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
#include "gazebo/gui/ApplyWrenchDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ApplyWrenchDialog::ApplyWrenchDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ApplyWrenchDialog");

  this->setWindowTitle(tr("Apply Force and Torque"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);
  this->setWindowModality(Qt::NonModal);

  this->messageLabel = new QLabel();
  this->messageLabel->setText(
      tr("Apply Force and Torque"));

  // Reference point
  QRadioButton *pointCollapser = new QRadioButton();
  pointCollapser->setChecked(false);
  pointCollapser->setText("Reference Point");
  pointCollapser->setStyleSheet(
     "QRadioButton {\
        color: #d0d0d0;\
      }\
      QRadioButton::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QRadioButton::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(pointCollapser, SIGNAL(toggled(bool)), this,
           SLOT(TogglePoint(bool)));

  // Point X
  QLabel *pointXLabel = new QLabel();
  pointXLabel->setText(tr("X:"));
  QLabel *pointXUnitLabel = new QLabel();
  pointXUnitLabel->setText(tr("m"));

  this->pointXSpin = new QDoubleSpinBox();
  this->pointXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->pointXSpin->setSingleStep(0.1);
  this->pointXSpin->setDecimals(3);
  this->pointXSpin->setValue(0);
  connect(this->pointXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointXChanged(double)));

  // Point Y
  QLabel *pointYLabel = new QLabel();
  pointYLabel->setText(tr("Y:"));
  QLabel *pointYUnitLabel = new QLabel();
  pointYUnitLabel->setText(tr("m"));

  this->pointYSpin = new QDoubleSpinBox();
  this->pointYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->pointYSpin->setSingleStep(0.1);
  this->pointYSpin->setDecimals(3);
  this->pointYSpin->setValue(0);
  connect(this->pointYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointYChanged(double)));

  // Point Z
  QLabel *pointZLabel = new QLabel();
  pointZLabel->setText(tr("Z:"));
  QLabel *pointZUnitLabel = new QLabel();
  pointZUnitLabel->setText(tr("m"));

  this->pointZSpin = new QDoubleSpinBox();
  this->pointZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->pointZSpin->setSingleStep(0.1);
  this->pointZSpin->setDecimals(3);
  this->pointZSpin->setValue(0);
  connect(this->pointZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnPointZChanged(double)));

  QGridLayout *pointCollapsible = new QGridLayout();
  pointCollapsible->addWidget(pointXLabel, 0, 0);
  pointCollapsible->addWidget(this->pointXSpin, 0, 1);
  pointCollapsible->addWidget(pointXUnitLabel, 0, 2);
  pointCollapsible->addWidget(pointYLabel, 1, 0);
  pointCollapsible->addWidget(this->pointYSpin, 1, 1);
  pointCollapsible->addWidget(pointYUnitLabel, 1, 2);
  pointCollapsible->addWidget(pointZLabel, 2, 0);
  pointCollapsible->addWidget(this->pointZSpin, 2, 1);
  pointCollapsible->addWidget(pointZUnitLabel, 2, 2);

  this->pointCollapsibleWidget = new QWidget();
  this->pointCollapsibleWidget->setLayout(pointCollapsible);
  this->pointCollapsibleWidget->hide();

  QVBoxLayout *pointLayout = new QVBoxLayout();
  pointLayout->addWidget(pointCollapser);
  pointLayout->addWidget(pointCollapsibleWidget);

  // Force
  QLabel *forceLabel = new QLabel();
  forceLabel->setText(tr("Force"));

  // Force magnitude
  QLabel *forceMagLabel = new QLabel();
  forceMagLabel->setText(tr("Total:"));
  QLabel *forceMagUnitLabel = new QLabel();
  forceMagUnitLabel->setText(tr("N"));

  this->forceMagSpin = new QDoubleSpinBox();
  this->forceMagSpin->setRange(0, GZ_DBL_MAX);
  this->forceMagSpin->setSingleStep(0.1);
  this->forceMagSpin->setDecimals(3);
  this->forceMagSpin->setValue(1000);
  connect(this->forceMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceMagChanged(double)));

  // Force X
  QLabel *forceXLabel = new QLabel();
  forceXLabel->setText(tr("X:"));
  QLabel *forceXUnitLabel = new QLabel();
  forceXUnitLabel->setText(tr("N"));

  this->forceXSpin = new QDoubleSpinBox();
  this->forceXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->forceXSpin->setSingleStep(0.1);
  this->forceXSpin->setDecimals(3);
  this->forceXSpin->setValue(1);
  connect(this->forceXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceXChanged(double)));

  // Force Y
  QLabel *forceYLabel = new QLabel();
  forceYLabel->setText(tr("Y:"));
  QLabel *forceYUnitLabel = new QLabel();
  forceYUnitLabel->setText(tr("N"));

  this->forceYSpin = new QDoubleSpinBox();
  this->forceYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->forceYSpin->setSingleStep(0.1);
  this->forceYSpin->setDecimals(3);
  this->forceYSpin->setValue(0);
  connect(this->forceYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceYChanged(double)));

  // Force Z
  QLabel *forceZLabel = new QLabel();
  forceZLabel->setText(tr("Z:"));
  QLabel *forceZUnitLabel = new QLabel();
  forceZUnitLabel->setText(tr("N"));

  this->forceZSpin = new QDoubleSpinBox();
  this->forceZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->forceZSpin->setSingleStep(0.1);
  this->forceZSpin->setDecimals(3);
  this->forceZSpin->setValue(0);
  connect(this->forceZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnForceZChanged(double)));

  QGridLayout *forceLayout = new QGridLayout();
  forceLayout->addWidget(forceLabel, 0, 0);
  forceLayout->addWidget(forceXLabel, 1, 0);
  forceLayout->addWidget(this->forceXSpin, 1, 1);
  forceLayout->addWidget(forceXUnitLabel, 1, 2);
  forceLayout->addWidget(forceYLabel, 2, 0);
  forceLayout->addWidget(this->forceYSpin, 2, 1);
  forceLayout->addWidget(forceYUnitLabel, 2, 2);
  forceLayout->addWidget(forceZLabel, 3, 0);
  forceLayout->addWidget(this->forceZSpin, 3, 1);
  forceLayout->addWidget(forceZUnitLabel, 3, 2);
  forceLayout->addWidget(forceMagLabel, 4, 0);
  forceLayout->addWidget(this->forceMagSpin, 4, 1);
  forceLayout->addWidget(forceMagUnitLabel, 4, 2);

  // Torque
  QLabel *torqueLabel = new QLabel();
  torqueLabel->setText(tr("Torque"));

  // Torque magnitude
  QLabel *torqueMagLabel = new QLabel();
  torqueMagLabel->setText(tr("Total:"));
  QLabel *torqueMagUnitLabel = new QLabel();
  torqueMagUnitLabel->setText(tr("Nm"));

  this->torqueMagSpin = new QDoubleSpinBox();
  this->torqueMagSpin->setRange(0, GZ_DBL_MAX);
  this->torqueMagSpin->setSingleStep(0.1);
  this->torqueMagSpin->setDecimals(3);
  this->torqueMagSpin->setValue(0);
  connect(this->torqueMagSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueMagChanged(double)));

  // Torque X
  QLabel *torqueXLabel = new QLabel();
  torqueXLabel->setText(tr("X:"));
  QLabel *torqueXUnitLabel = new QLabel();
  torqueXUnitLabel->setText(tr("Nm"));

  this->torqueXSpin = new QDoubleSpinBox();
  this->torqueXSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->torqueXSpin->setSingleStep(0.1);
  this->torqueXSpin->setDecimals(3);
  this->torqueXSpin->setValue(0);
  connect(this->torqueXSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueXChanged(double)));

  // Torque Y
  QLabel *torqueYLabel = new QLabel();
  torqueYLabel->setText(tr("Y:"));
  QLabel *torqueYUnitLabel = new QLabel();
  torqueYUnitLabel->setText(tr("Nm"));

  this->torqueYSpin = new QDoubleSpinBox();
  this->torqueYSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->torqueYSpin->setSingleStep(0.1);
  this->torqueYSpin->setDecimals(3);
  this->torqueYSpin->setValue(0);
  connect(this->torqueYSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueYChanged(double)));

  // Torque Z
  QLabel *torqueZLabel = new QLabel();
  torqueZLabel->setText(tr("Z:"));
  QLabel *torqueZUnitLabel = new QLabel();
  torqueZUnitLabel->setText(tr("Nm"));

  this->torqueZSpin = new QDoubleSpinBox();
  this->torqueZSpin->setRange(-GZ_DBL_MAX, GZ_DBL_MAX);
  this->torqueZSpin->setSingleStep(0.1);
  this->torqueZSpin->setDecimals(3);
  this->torqueZSpin->setValue(0);
  connect(this->torqueZSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnTorqueZChanged(double)));

  QGridLayout *torqueLayout = new QGridLayout();
  torqueLayout->addWidget(torqueLabel, 0, 0);
  torqueLayout->addWidget(torqueXLabel, 1, 0);
  torqueLayout->addWidget(this->torqueXSpin, 1, 1);
  torqueLayout->addWidget(torqueXUnitLabel, 1, 2);
  torqueLayout->addWidget(torqueYLabel, 2, 0);
  torqueLayout->addWidget(this->torqueYSpin, 2, 1);
  torqueLayout->addWidget(torqueYUnitLabel, 2, 2);
  torqueLayout->addWidget(torqueZLabel, 3, 0);
  torqueLayout->addWidget(this->torqueZSpin, 3, 1);
  torqueLayout->addWidget(torqueZUnitLabel, 3, 2);
  torqueLayout->addWidget(torqueMagLabel, 4, 0);
  torqueLayout->addWidget(this->torqueMagSpin, 4, 1);
  torqueLayout->addWidget(torqueMagUnitLabel, 4, 2);

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
  mainLayout->addWidget(this->messageLabel);
//  mainLayout->addLayout(pointLayout);
  mainLayout->addLayout(forceLayout);
  mainLayout->addLayout(torqueLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->UpdateForceVector();
  this->UpdateTorqueVector();
  this->CalculateWrench();
  this->SetMode(rendering::ApplyWrenchVisual::FORCE);

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
  this->node->Fini();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::SetLink(std::string _linkName)
{
  std::string msg = "Apply Force and Torque\n\nApply to " + _linkName + "\n";
  this->messageLabel->setText(msg.c_str());
  this->linkName = _linkName;
  this->SetPublisher();

  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
        GetVisual(this->linkName);

  this->linkVisual = vis;
  this->AttachVisuals();
}

///////////////////////////////////////////////////
void ApplyWrenchDialog::SetMode(rendering::ApplyWrenchVisual::WrenchModes _mode)
{
  this->wrenchMode = _mode;
  if (this->applyWrenchVisual)
    this->applyWrenchVisual->SetMode(this->wrenchMode);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnApply()
{
  // publish wrench msg
  msgs::Wrench msg;
  msgs::Set(msg.mutable_force(), this->forceVector);
  msgs::Set(msg.mutable_torque(), this->torqueVector);

  this->wrenchPub->Publish(msg);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnCancel()
{
  this->applyWrenchVisual->SetVisible(false);
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
  this->SetMode(rendering::ApplyWrenchVisual::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceXChanged(double /*_fX*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceYChanged(double /*_fY*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnForceZChanged(double /*_fZ*/)
{
  this->UpdateForceMag();
  this->SetMode(rendering::ApplyWrenchVisual::FORCE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueMagChanged(double /*_magnitude*/)
{
  this->UpdateTorqueVector();
  this->SetMode(rendering::ApplyWrenchVisual::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueXChanged(double /*_fX*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueYChanged(double /*_fY*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::TORQUE);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::OnTorqueZChanged(double /*_fZ*/)
{
  this->UpdateTorqueMag();
  this->SetMode(rendering::ApplyWrenchVisual::TORQUE);
}

//////////////////////////////////////////////////
void ApplyWrenchDialog::SetPublisher()
{
  this->topicName = "~/";
  topicName += this->linkName + "/wrench";
  boost::replace_all(topicName, "::", "/");

  this->wrenchPub = this->node->Advertise<msgs::Wrench>(this->topicName);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::CalculateWrench()
{
  this->forceVector =
      math::Vector3(this->forceXSpin->value(),
                    this->forceYSpin->value(),
                    this->forceZSpin->value());
  this->torqueVector =
      math::Vector3(this->torqueXSpin->value(),
                    this->torqueYSpin->value(),
                    this->torqueZSpin->value());

  // Update visuals
  if (this->applyWrenchVisual)
  {
    this->applyWrenchVisual->UpdateForce(this->forceVector);
    this->applyWrenchVisual->UpdateTorque(this->torqueVector);
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceMag()
{
  this->forceMagSpin->setValue(sqrt(
      pow(this->forceXSpin->value(), 2) +
      pow(this->forceYSpin->value(), 2) +
      pow(this->forceZSpin->value(), 2)));
  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->forceXSpin->value(),
                                   this->forceYSpin->value(),
                                   this->forceZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->forceMagSpin->value();

  // Update spins
  this->forceXSpin->setValue(v.x);
  this->forceYSpin->setValue(v.y);
  this->forceZSpin->setValue(v.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceVector(math::Vector3 _fV)
{
  // Normalize new vector;
  if (_fV == math::Vector3::Zero)
    _fV = math::Vector3::UnitX;
  else
    _fV.Normalize();

  // Multiply by magnitude
  _fV = _fV * this->forceMagSpin->value();

  // Update spins
  this->forceXSpin->setValue(_fV.x);
  this->forceYSpin->setValue(_fV.y);
  this->forceZSpin->setValue(_fV.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueMag()
{
  this->torqueMagSpin->setValue(sqrt(
      pow(this->torqueXSpin->value(), 2) +
      pow(this->torqueYSpin->value(), 2) +
      pow(this->torqueZSpin->value(), 2)));
  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVector()
{
  // Normalize current vector
  math::Vector3 v = math::Vector3(this->torqueXSpin->value(),
                                   this->torqueYSpin->value(),
                                   this->torqueZSpin->value());
  if (v == math::Vector3::Zero)
    v = math::Vector3::UnitX;
  else
    v.Normalize();

  // Multiply by new magnitude
  v = v * this->torqueMagSpin->value();

  // Update spins
  this->torqueXSpin->setValue(v.x);
  this->torqueYSpin->setValue(v.y);
  this->torqueZSpin->setValue(v.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueVector(math::Vector3 _tV)
{
  // Normalize new vector;
  if (_tV == math::Vector3::Zero)
    _tV = math::Vector3::UnitX;
  else
    _tV.Normalize();

  // Multiply by magnitude
  _tV = _tV * this->torqueMagSpin->value();

  // Update spins
  this->torqueXSpin->setValue(_tV.x);
  this->torqueYSpin->setValue(_tV.y);
  this->torqueZSpin->setValue(_tV.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::AttachVisuals()
{
  if (!this->applyWrenchVisual)
  {
    this->applyWrenchVisual.reset(new rendering::ApplyWrenchVisual(this->linkName +
        "__APPLY_WRENCH__", this->linkVisual));

    this->applyWrenchVisual->Load();
    this->applyWrenchVisual->SetMode(this->wrenchMode);
  }
  else
  {
    this->applyWrenchVisual->SetVisible(true);
  }

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::TogglePoint(bool _checked)
{
  if (_checked)
  {
    this->pointCollapsibleWidget->show();
    //this->resize(this->maximumSize());
  }
  else
  {
    this->pointCollapsibleWidget->hide();
    //this->resize(this->minimumSize());
  }
}

/////////////////////////////////////////////////
bool ApplyWrenchDialog::OnMousePress(const common::MouseEvent & _event)
{
  rendering::UserCameraPtr userCamera = gui::get_active_camera();
  if (!userCamera)
    return false;

  this->dragStart = math::Vector2i(0, 0);

  std::string manipState;
  rendering::VisualPtr vis = userCamera->GetVisual(_event.pos, manipState);
  this->applyWrenchVisual->GetRotTool()->SetState(manipState);

  if (!vis)
    return false;

//  if (manipState == rendering::SelectionObj::ROT_X ||
//      manipState == rendering::SelectionObj::ROT_Y)
  {
    this->dragStart = _event.pressPos;
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

  // Dragging
  if (_event.dragging && _event.button == common::MouseEvent::LEFT &&
      this->dragStart.x != 0)
  {
    // TODO: calculate proper displacement
    math::Vector3 vec = math::Vector3(-1, -1, -1);

    if (this->wrenchMode == rendering::ApplyWrenchVisual::FORCE)
    {
      this->UpdateForceVector(vec);
    }
    else if (this->wrenchMode == rendering::ApplyWrenchVisual::TORQUE)
    {
      this->UpdateTorqueVector(vec);
    }
  }
  // Highlight hovered tools
  else
  {
    std::string manipState;
    rendering::VisualPtr vis = userCamera->GetVisual(_event.pos, manipState);
    this->applyWrenchVisual->GetRotTool()->SetState(manipState);

    if (!vis)
      return false;
  }

  return false;
}
