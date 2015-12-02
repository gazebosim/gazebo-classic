/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/ConfigWidget.hh"

#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/CollisionConfig.hh"
#include "gazebo/gui/model/LinkInspector.hh"

#include "gazebo/math/Helpers.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LinkInspector::LinkInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("LinkInspector");
  this->setWindowTitle(tr("Link Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QLabel *linkLabel = new QLabel(tr("Name:"));
  this->linkNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(linkLabel);
  nameLayout->addWidget(this->linkNameLabel, QSizePolicy::Maximum);
  nameLayout->setAlignment(this->linkNameLabel, Qt::AlignLeft);

  this->linkConfig = new LinkConfig;
  connect(this->linkConfig, SIGNAL(Applied()), this, SLOT(OnConfigApplied()));
  this->visualConfig = new VisualConfig;
  connect(this->visualConfig, SIGNAL(Applied()), this, SLOT(OnConfigApplied()));
  this->collisionConfig = new CollisionConfig;
  connect(this->collisionConfig, SIGNAL(Applied()), this,
      SLOT(OnConfigApplied()));

  // Create the main tab widget for all components in a link
  this->tabWidget = new QTabWidget();
  this->tabWidget->setObjectName("linkInspectorTab");
  this->tabWidget->setMinimumHeight(300);
  this->tabWidget->setMinimumWidth(560);

  this->tabWidget->addTab(this->linkConfig, "Link");
  this->tabWidget->addTab(this->visualConfig, "Visual");
  this->tabWidget->addTab(this->collisionConfig, "Collision");

  // Buttons
  QToolButton *removeButton = new QToolButton(this);
  removeButton->setFixedSize(QSize(30, 30));
  removeButton->setToolTip("Remove link");
  removeButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeButton->setIconSize(QSize(16, 16));
  removeButton->setCheckable(false);
  connect(removeButton, SIGNAL(clicked()), this, SLOT(OnRemove()));

  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(RestoreOriginalData()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *applyButton = new QPushButton(tr("Apply"));
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));

  QPushButton *OKButton = new QPushButton(tr("OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(removeButton);
  buttonsLayout->addStretch(5);
  buttonsLayout->addWidget(resetButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(nameLayout);
  mainLayout->addWidget(tabWidget);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);

  // Conections
  connect(this, SIGNAL(rejected()), this, SLOT(RestoreOriginalData()));

  connect(this->linkConfig, SIGNAL(DensityValueChanged(const double &)),
      this, SLOT(OnDensityValueChanged(const double &)));

  connect(this->linkConfig, SIGNAL(MassValueChanged(const double &)),
      this, SLOT(OnMassValueChanged(const double &)));

  connect(
      this->collisionConfig,
      SIGNAL(CollisionChanged(const std::string &, const std::string &)),
      this,
      SLOT(OnCollisionChanged(const std::string &, const std::string &)));
}

/////////////////////////////////////////////////
LinkInspector::~LinkInspector()
{
  delete this->linkConfig;
  this->linkConfig = NULL;
  delete this->visualConfig;
  this->visualConfig = NULL;
  delete this->collisionConfig;
  this->collisionConfig = NULL;
}

/////////////////////////////////////////////////
void LinkInspector::SetName(const std::string &_name)
{
  this->linkNameLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string LinkInspector::GetName() const
{
  return this->linkNameLabel->text().toStdString();
}

/////////////////////////////////////////////////
LinkConfig *LinkInspector::GetLinkConfig() const
{
  return this->linkConfig;
}

/////////////////////////////////////////////////
VisualConfig *LinkInspector::GetVisualConfig() const
{
  return this->visualConfig;
}

/////////////////////////////////////////////////
CollisionConfig *LinkInspector::GetCollisionConfig() const
{
  return this->collisionConfig;
}

/////////////////////////////////////////////////
void LinkInspector::OnRemove()
{
  this->close();

  model::Events::requestLinkRemoval(this->linkId);
}

/////////////////////////////////////////////////
void LinkInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void LinkInspector::OnConfigApplied()
{
  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::OnOK()
{
  emit Accepted();
}

/////////////////////////////////////////////////
void LinkInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void LinkInspector::SetLinkId(const std::string &_id)
{
  this->linkId = _id;
}

/////////////////////////////////////////////////
void LinkInspector::Open()
{
  this->linkConfig->Init();
  this->visualConfig->Init();
  this->collisionConfig->Init();

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void LinkInspector::RestoreOriginalData()
{
  this->linkConfig->RestoreOriginalData();
  this->visualConfig->RestoreOriginalData();
  this->collisionConfig->RestoreOriginalData();

  emit Applied();
}

/////////////////////////////////////////////////
void LinkInspector::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Enter)
    _event->accept();
  else
    QDialog::keyPressEvent(_event);
}

/////////////////////////////////////////////////
double LinkInspector::ComputeVolume() const
{
  double volume = 0;

  for (auto it : this->collisionConfig->ConfigData())
  {
    msgs::Collision *coll = this->collisionConfig->GetData(it.second->name);
    if (coll)
      volume += LinkData::ComputeVolume(*coll);
  }
  return volume;
}

/////////////////////////////////////////////////
ignition::math::Vector3d LinkInspector::ComputeInertia(double _mass) const
{
  ignition::math::Vector3d I = ignition::math::Vector3d::Zero;

  // Use first collision entry
  for (auto it : this->collisionConfig->ConfigData())
  {
    msgs::Collision *coll = this->collisionConfig->GetData(it.second->name);
    if (coll)
    {
      I = gui::LinkData::ComputeMomentOfInertia(*coll, _mass);
      break;
    }
  }
  return I;
}

/////////////////////////////////////////////////
void LinkInspector::OnDensityValueChanged(const double &_value)
{
  double volume = ComputeVolume();
  double mass = volume * _value;

  if (!ignition::math::equal(this->linkConfig->Mass(), mass))
  {
    ignition::math::Vector3d I = ComputeInertia(mass);
    this->linkConfig->SetMass(mass);
    this->linkConfig->SetInertiaMatrix(I.X(), I.Y(), I.Z(), 0, 0, 0);
  }
}

/////////////////////////////////////////////////
void LinkInspector::OnMassValueChanged(const double &_value)
{
  double volume = ComputeVolume();

  if (!math::equal(volume, 0.0))
  {
    double density = _value / volume;
    if (!math::equal(this->linkConfig->Density(), density))
    {
      ignition::math::Vector3d I = ComputeInertia(_value);
      this->linkConfig->SetDensity(density);
      this->linkConfig->SetInertiaMatrix(I.X(), I.Y(), I.Z(), 0, 0, 0);
    }
  }
}

/////////////////////////////////////////////////
void LinkInspector::OnCollisionChanged(const std::string &/*_name*/,
    const std::string &_type)
{
  if (_type == "geometry")
  {
    double volume = ComputeVolume();

    if (!math::equal(volume, 0.0))
    {
      double mass = this->linkConfig->Mass();
      double density = mass / volume;

      this->linkConfig->SetDensity(density);
    }
  }
}

