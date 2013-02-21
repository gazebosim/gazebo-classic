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

  connect(this, SIGNAL(AddPoint(const QPointF &)),
          this->plot, SLOT(Add(const QPointF &)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
Diagnostics::~Diagnostics()
{
}

/////////////////////////////////////////////////
void Diagnostics::OnMsg(ConstDiagnosticsPtr &_msg)
{
  if (this->paused)
    return;

  if (_msg->time_size() == 0)
    return;

  if (this->startTime == common::Time::Zero)
    this->startTime = msgs::Convert(_msg->time(0).wall());

  common::Time wallTime, elapsedTime;

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

    if (this->selectedLabel == _msg->time(i).name())
    {
      wallTime = msgs::Convert(_msg->time(i).wall());
      elapsedTime = msgs::Convert(_msg->time(i).elapsed());

      double msTime = elapsedTime.Double() * 1e3;
      QPointF pt((wallTime - this->startTime).Double(), msTime);

      this->AddPoint(pt);
    }
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnLabelSelected(QListWidgetItem *_item)
{
  this->selectedLabel = _item->text().toStdString();
  this->plot->Clear();
}

/////////////////////////////////////////////////
void Diagnostics::OnPause(bool _value)
{
  this->paused = _value;
}
