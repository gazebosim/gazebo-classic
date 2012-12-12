/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "common/SystemPaths.hh"

#include "gui/Gui.hh"
#include "gui/GuiEvents.hh"

#include "transport/Node.hh"
#include "transport/Publisher.hh"

#include "gui/SkyWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
SkyWidget::SkyWidget(QWidget *_parent)
: QWidget(_parent)
{
  this->setObjectName("sky");

  QLabel *label;
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;


  QHBoxLayout *timeLayout = new QHBoxLayout;
  this->timeLineEdit = new QDoubleSpinBox;
  this->timeLineEdit->setRange(0, 24);
  this->timeLineEdit->setSingleStep(0.1);
  this->timeLineEdit->setDecimals(2);
  this->timeLineEdit->setValue(10.0);
  label = new QLabel(tr("Time:"));
  timeLayout->addWidget(label);
  timeLayout->addWidget(this->timeLineEdit);
  timeLayout->insertStretch(1, 0);


  QHBoxLayout *sunriseLayout = new QHBoxLayout;
  this->sunriseLineEdit = new QDoubleSpinBox;
  this->sunriseLineEdit->setRange(0, 24);
  this->sunriseLineEdit->setSingleStep(0.1);
  this->sunriseLineEdit->setDecimals(2);
  this->sunriseLineEdit->setValue(10.0);
  label = new QLabel(tr("Sunrise:"));
  sunriseLayout->addWidget(label);
  sunriseLayout->addWidget(this->sunriseLineEdit);
  sunriseLayout->insertStretch(1, 0);


  QHBoxLayout *sunsetLayout = new QHBoxLayout;
  this->sunsetLineEdit = new QDoubleSpinBox;
  this->sunsetLineEdit->setRange(0, 24);
  this->sunsetLineEdit->setSingleStep(0.1);
  this->sunsetLineEdit->setDecimals(2);
  this->sunsetLineEdit->setValue(10.0);
  label = new QLabel(tr("Sunset:"));
  sunsetLayout->addWidget(label);
  sunsetLayout->addWidget(this->sunsetLineEdit);
  sunsetLayout->insertStretch(1, 0);


  // Clouds
  // Wind Speed
  QHBoxLayout *windSpeedLayout = new QHBoxLayout;
  this->windSpeedLineEdit = new QDoubleSpinBox;
  this->windSpeedLineEdit->setRange(0, 10);
  this->windSpeedLineEdit->setSingleStep(0.1);
  this->windSpeedLineEdit->setDecimals(2);
  this->windSpeedLineEdit->setValue(10.0);
  label = new QLabel(tr("Wind Speed:"));
  windSpeedLayout->addWidget(label);
  windSpeedLayout->addWidget(this->windSpeedLineEdit);
  windSpeedLayout->insertStretch(1, 0);


  // Direction
  QHBoxLayout *windDirectionLayout = new QHBoxLayout;
  this->windDirectionLineEdit = new QDoubleSpinBox;
  this->windDirectionLineEdit->setRange(0, M_PI*2.0);
  this->windDirectionLineEdit->setSingleStep(0.1);
  this->windDirectionLineEdit->setDecimals(2);
  this->windDirectionLineEdit->setValue(10.0);
  label = new QLabel(tr("Wind Direction:"));
  windDirectionLayout->addWidget(label);
  windDirectionLayout->addWidget(this->windDirectionLineEdit);
  windDirectionLayout->insertStretch(1, 0);

  QHBoxLayout *cloudAmbientColorLayout = new QHBoxLayout;
  this->cloudAmbientColorButton = new QPushButton;
  label = new QLabel(tr("Color:"));
  cloudAmbientColorLayout->addWidget(label);
  cloudAmbientColorLayout->addWidget(this->cloudAmbientColorButton);
  cloudAmbientColorLayout->insertStretch(1, 0);

  QHBoxLayout *humidityLayout = new QHBoxLayout;
  this->humidityLineEdit = new QDoubleSpinBox;
  this->humidityLineEdit->setRange(0, 1);
  this->humidityLineEdit->setSingleStep(0.1);
  this->humidityLineEdit->setDecimals(2);
  this->humidityLineEdit->setValue(10.0);
  label = new QLabel(tr("Humidity:"));
  humidityLayout->addWidget(label);
  humidityLayout->addWidget(this->humidityLineEdit);
  humidityLayout->insertStretch(1, 0);

  QHBoxLayout *avgCloudSizeLayout = new QHBoxLayout;
  this->avgCloudSizeLineEdit = new QDoubleSpinBox;
  this->avgCloudSizeLineEdit->setRange(0, 1);
  this->avgCloudSizeLineEdit->setSingleStep(0.1);
  this->avgCloudSizeLineEdit->setDecimals(2);
  this->avgCloudSizeLineEdit->setValue(10.0);
  label = new QLabel(tr("Mean Cloud Size:"));
  avgCloudSizeLayout->addWidget(label);
  avgCloudSizeLayout->addWidget(this->avgCloudSizeLineEdit);
  avgCloudSizeLayout->insertStretch(1, 0);

  frameLayout->addLayout(timeLayout);
  frameLayout->addLayout(sunriseLayout);
  frameLayout->addLayout(sunsetLayout);
  frameLayout->addLayout(windSpeedLayout);
  frameLayout->addLayout(windDirectionLayout);
  frameLayout->addLayout(cloudAmbientColorLayout);
  frameLayout->addLayout(humidityLayout);
  frameLayout->addLayout(avgCloudSizeLayout);

  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  connect(this->timeLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnTime(double)));
  connect(this->sunriseLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnSunrise(double)));
  connect(this->sunsetLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnSunset(double)));
  connect(this->windSpeedLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnWindSpeed(double)));
  connect(this->windDirectionLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnWindDirection(double)));
  connect(this->humidityLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnHumidity(double)));
  connect(this->avgCloudSizeLineEdit, SIGNAL(valueChanged(double)),
          this, SLOT(OnAvgCloudSize(double)));


  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->pub = this->node->Advertise<msgs::Sky>("~/sky");
}

/////////////////////////////////////////////////
SkyWidget::~SkyWidget()
{
}

/////////////////////////////////////////////////
void SkyWidget::OnTime(double _v)
{
  msgs::Sky msg;
  msg.set_time(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnSunrise(double _v)
{
  msgs::Sky msg;
  msg.set_sunrise(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnSunset(double _v)
{
  msgs::Sky msg;
  msg.set_sunset(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnWindSpeed(double _v)
{
  msgs::Sky msg;
  msg.set_wind_speed(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnWindDirection(double _v)
{
  msgs::Sky msg;
  msg.set_wind_direction(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnHumidity(double _v)
{
  msgs::Sky msg;
  msg.set_humidity(_v);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void SkyWidget::OnAvgCloudSize(double _v)
{
  msgs::Sky msg;
  msg.set_mean_cloud_size(_v);
  this->pub->Publish(msg);
}
