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

#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/IncrementalPlot.hh"
#include "gazebo/gui/Diagnostics.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
Diagnostics::Diagnostics(QWidget *_parent)
  : QDialog(_parent)
{
  this->paused = false;
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Diagnostics");
  this->setObjectName("diagnosticsPlot");

  this->plot = new IncrementalPlot(this);

  this->labelList = new QListWidget;
  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  this->labelList->addItem(item);
  connect(this->labelList, SIGNAL(itemClicked(QListWidgetItem *)),
          this, SLOT(OnLabelSelected(QListWidgetItem *)));

  QCheckBox *pauseCheck = new QCheckBox;
  pauseCheck->setChecked(this->paused);
  connect(pauseCheck, SIGNAL(clicked(bool)), this, SLOT(OnPause(bool)));

  QHBoxLayout *pauseLayout = new QHBoxLayout;
  pauseLayout->addWidget(pauseCheck);
  pauseLayout->addWidget(new QLabel("Pause"));
  pauseLayout->addStretch(1);

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addWidget(this->labelList);
  leftLayout->addLayout(pauseLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(leftLayout);
  mainLayout->addWidget(this->plot, 2);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->sub = this->node->Subscribe("~/diagnostics", &Diagnostics::OnMsg, this);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(60);
}

/////////////////////////////////////////////////
Diagnostics::~Diagnostics()
{
}

/////////////////////////////////////////////////
void Diagnostics::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  for (PointMap::iterator iter = this->selectedLabels.begin();
       iter != this->selectedLabels.end(); ++iter)
  {
    this->plot->Add(iter->first, iter->second);
    iter->second.clear();
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnMsg(ConstDiagnosticsPtr &_msg)
{
  if (this->paused)
    return;

  if (_msg->time_size() == 0)
    return;

  boost::mutex::scoped_lock lock(this->mutex);

  if (this->startTime == common::Time::Zero)
    this->startTime = msgs::Convert(_msg->time(0).wall());

  common::Time wallTime, elapsedTime;
  PointMap::iterator labelIter;

  if (this->selectedLabels.find(QString("Real Time Factor")) !=
      this->selectedLabels.end())
  {

    wallTime = msgs::Convert(_msg->real_time());
    this->selectedLabels[QString("Real Time Factor")].push_back(
        QPointF(wallTime.Double(), _msg->real_time_factor()));
  }

  for (int i = 0; i < _msg->time_size(); ++i)
  {
    QString qstr = QString::fromStdString(_msg->time(i).name());

    QList<QListWidgetItem*> items = this->labelList->findItems(qstr,
        Qt::MatchExactly);

    if (items.size() == 0)
    {
      QListWidgetItem *item = new QListWidgetItem(qstr);
      this->labelList->addItem(item);
    }

    labelIter = this->selectedLabels.find(
        QString::fromStdString(_msg->time(i).name()));

    if (labelIter != this->selectedLabels.end())
    {
      wallTime = msgs::Convert(_msg->time(i).wall());
      elapsedTime = msgs::Convert(_msg->time(i).elapsed());

      double msTime = elapsedTime.Double() * 1e3;
      QPointF pt((wallTime - this->startTime).Double(), msTime);

      labelIter->second.push_back(pt);
    }
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnLabelSelected(QListWidgetItem *_item)
{
  // Add the label if it doens't exist. Otherwise remove the label
  if (this->selectedLabels.find(_item->text()) ==
      this->selectedLabels.end())
  {
    this->selectedLabels[_item->text()].clear();
  }
  else
    this->selectedLabels.erase(_item->text());
}

/////////////////////////////////////////////////
void Diagnostics::OnPause(bool _value)
{
  this->paused = _value;
}
