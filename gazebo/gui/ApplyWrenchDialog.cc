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
  QLabel *forceLabel = new QLabel();
  forceLabel->setText(tr("Force"));

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

  QGridLayout *forceLayout = new QGridLayout();
  forceLayout->addWidget(forceLabel, 0, 0);
  forceLayout->addWidget(forceXLabel, 1, 0);
  forceLayout->addWidget(this->dataPtr->forceXSpin, 1, 1);
  forceLayout->addWidget(forceXUnitLabel, 1, 2);
  forceLayout->addWidget(forceYLabel, 2, 0);
  forceLayout->addWidget(this->dataPtr->forceYSpin, 2, 1);
  forceLayout->addWidget(forceYUnitLabel, 2, 2);
  forceLayout->addWidget(forceZLabel, 3, 0);
  forceLayout->addWidget(this->dataPtr->forceZSpin, 3, 1);
  forceLayout->addWidget(forceZUnitLabel, 3, 2);
  forceLayout->addWidget(forceMagLabel, 4, 0);
  forceLayout->addWidget(this->dataPtr->forceMagSpin, 4, 1);
  forceLayout->addWidget(forceMagUnitLabel, 4, 2);

  // Torque
  QLabel *torqueLabel = new QLabel();
  torqueLabel->setText(tr("Torque"));

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

  QGridLayout *torqueLayout = new QGridLayout();
  torqueLayout->addWidget(torqueLabel, 0, 0);
  torqueLayout->addWidget(torqueXLabel, 1, 0);
  torqueLayout->addWidget(this->dataPtr->torqueXSpin, 1, 1);
  torqueLayout->addWidget(torqueXUnitLabel, 1, 2);
  torqueLayout->addWidget(torqueYLabel, 2, 0);
  torqueLayout->addWidget(this->dataPtr->torqueYSpin, 2, 1);
  torqueLayout->addWidget(torqueYUnitLabel, 2, 2);
  torqueLayout->addWidget(torqueZLabel, 3, 0);
  torqueLayout->addWidget(this->dataPtr->torqueZSpin, 3, 1);
  torqueLayout->addWidget(torqueZUnitLabel, 3, 2);
  torqueLayout->addWidget(torqueMagLabel, 4, 0);
  torqueLayout->addWidget(this->dataPtr->torqueMagSpin, 4, 1);
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
  std::string topicName = "~/";
  topicName += this->dataPtr->linkName + "/wrench";
  boost::replace_all(topicName, "::", "/");

  this->dataPtr->wrenchPub = this->dataPtr->node->Advertise<msgs::Wrench>(topicName);
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::CalculateWrench()
{
  this->dataPtr->forceVector =
      math::Vector3(this->dataPtr->forceXSpin->value(),
                    this->dataPtr->forceYSpin->value(),
                    this->dataPtr->forceZSpin->value());
  this->dataPtr->torqueVector =
      math::Vector3(this->dataPtr->torqueXSpin->value(),
                    this->dataPtr->torqueYSpin->value(),
                    this->dataPtr->torqueZSpin->value());

  // Update visuals
  if (this->dataPtr->applyWrenchVisual)
  {
    this->dataPtr->applyWrenchVisual->UpdateForce(this->dataPtr->forceVector);
    this->dataPtr->applyWrenchVisual->UpdateTorque(this->dataPtr->torqueVector);
  }
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateForceMag()
{
  this->dataPtr->forceMagSpin->setValue(sqrt(
      pow(this->dataPtr->forceXSpin->value(), 2) +
      pow(this->dataPtr->forceYSpin->value(), 2) +
      pow(this->dataPtr->forceZSpin->value(), 2)));
  this->CalculateWrench();
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

  // Update spins
  this->dataPtr->forceXSpin->setValue(v.x);
  this->dataPtr->forceYSpin->setValue(v.y);
  this->dataPtr->forceZSpin->setValue(v.z);

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
  _fV = _fV * this->dataPtr->forceMagSpin->value();

  // Update spins
  this->dataPtr->forceXSpin->setValue(_fV.x);
  this->dataPtr->forceYSpin->setValue(_fV.y);
  this->dataPtr->forceZSpin->setValue(_fV.z);

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::UpdateTorqueMag()
{
  this->dataPtr->torqueMagSpin->setValue(sqrt(
      pow(this->dataPtr->torqueXSpin->value(), 2) +
      pow(this->dataPtr->torqueYSpin->value(), 2) +
      pow(this->dataPtr->torqueZSpin->value(), 2)));
  this->CalculateWrench();
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

  // Update spins
  this->dataPtr->torqueXSpin->setValue(v.x);
  this->dataPtr->torqueYSpin->setValue(v.y);
  this->dataPtr->torqueZSpin->setValue(v.z);

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
  _tV = _tV * this->dataPtr->torqueMagSpin->value();

  // Update spins
  this->dataPtr->torqueXSpin->setValue(_tV.x);
  this->dataPtr->torqueYSpin->setValue(_tV.y);
  this->dataPtr->torqueZSpin->setValue(_tV.z);

  this->CalculateWrench();
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

  this->CalculateWrench();
}

/////////////////////////////////////////////////
void ApplyWrenchDialog::TogglePoint(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->pointCollapsibleWidget->show();
    //this->resize(this->maximumSize());
  }
  else
  {
    this->dataPtr->pointCollapsibleWidget->hide();
    //this->resize(this->minimumSize());
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
  if (this->dataPtr->manipState == "rot_x" ||
      this->dataPtr->manipState == "rot_y")
  {
    this->dataPtr->applyWrenchVisual->GetRotTool()->SetState(
        this->dataPtr->manipState);
    this->dataPtr->dragStart = _event.pressPos;
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
    // TODO: calculate proper displacement

    math::Pose rotToolPose = this->dataPtr->applyWrenchVisual->GetRotTool()
        ->GetWorldPose();

    math::Vector3 normal;
    if (this->dataPtr->manipState == "rot_x")
    {
      normal = math::Vector3::UnitZ;
    }
    else if (this->dataPtr->manipState == "rot_y")
    {
      normal = math::Vector3::UnitY;
    }
    else
    {
      gzerr << "Manipulation state not supported." << std::endl;
      return false;
    }

    double offset = rotToolPose.pos.Dot(normal);

    math::Vector3 pressPoint;
    userCamera->GetWorldPointOnPlane(_event.pressPos.x, _event.pressPos.y,
          math::Plane(normal, offset), pressPoint);

    math::Vector3 newPoint;
    userCamera->GetWorldPointOnPlane(_event.pos.x, _event.pos.y,
        math::Plane(normal, offset), newPoint);

    math::Vector3 v1 = pressPoint - rotToolPose.pos;
    math::Vector3 v2 = newPoint - rotToolPose.pos;
    v1 = v1.Normalize();
    v2 = v2.Normalize();
    double signTest = v1.Cross(v2).Dot(normal);
    double angle = atan2((v1.Cross(v2)).GetLength(), v1.Dot(v2));

    if (signTest < 0 )
      angle *= -1;

    if (_event.control)
      angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

    math::Quaternion rot(normal, angle);

    math::Vector3 vec;
    if (this->dataPtr->manipState == "rot_x")
    {
      vec = rot.RotateVector(-math::Vector3::UnitY);
    }
    else
    {
      vec = rot.RotateVector(-math::Vector3::UnitZ);
    }
    //math::Vector3 vec = rot.RotateVector(-this->dataPtr->forceVector);
    if (this->dataPtr->wrenchMode == rendering::ApplyWrenchVisual::FORCE)
    {
      this->UpdateForceVector(vec);
    }
    else if (this->dataPtr->wrenchMode == rendering::ApplyWrenchVisual::TORQUE)
    {
      this->UpdateTorqueVector(vec);
    }
    return true;
  }
  // Highlight hovered tools
  else
  {
    userCamera->GetVisual(_event.pos, this->dataPtr->manipState);

    if (this->dataPtr->manipState == "rot_x" ||
      this->dataPtr->manipState == "rot_y")
    {
      this->dataPtr->applyWrenchVisual->GetRotTool()->SetState(
          this->dataPtr->manipState);
    }
  }

  return false;
}
