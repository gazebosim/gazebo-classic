/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/Diagnostics.hh"
#include "gazebo/gui/DiagnosticsPrivate.hh"
#include "gazebo/gui/IncrementalPlot.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/TransportIface.hh"

using namespace gazebo;
using namespace gui;

// A special list widget that allows dragging of items from it to a
// plot
class DragableListWidget : public QListWidget
{
  public: DragableListWidget(QWidget *_parent)
          : QListWidget(_parent)
          {
          }

  protected: virtual void startDrag(Qt::DropActions /*_supportedActions*/)
             {
               QListWidgetItem *currItem = this->currentItem();
               QMimeData *currMimeData = new QMimeData;
               QByteArray ba;
               ba = currItem->text().toLatin1().data();
               currMimeData->setData("application/x-item", ba);
               QDrag *drag = new QDrag(this);
               drag->setMimeData(currMimeData);
               drag->exec(Qt::LinkAction);
             }

  protected: virtual Qt::DropActions supportedDropActions()
             {
               return Qt::LinkAction;
             }
};

/////////////////////////////////////////////////
Diagnostics::Diagnostics(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new DiagnosticsPrivate())
{
  this->dataPtr->paused = false;
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Diagnostics");
  this->setObjectName("diagnosticsPlot");
  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  this->dataPtr->plotLayout = new QVBoxLayout;

  QScrollArea *plotScrollArea = new QScrollArea(this);
  plotScrollArea->setLineWidth(0);
  plotScrollArea->setFrameShape(QFrame::NoFrame);
  plotScrollArea->setFrameShadow(QFrame::Plain);
  plotScrollArea->setSizePolicy(QSizePolicy::Minimum,
                                QSizePolicy::Minimum);

  plotScrollArea->setWidgetResizable(true);
  plotScrollArea->viewport()->installEventFilter(this);

  QFrame *plotFrame = new QFrame(plotScrollArea);
  plotFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  plotFrame->setLayout(this->dataPtr->plotLayout);

  plotScrollArea->setWidget(plotFrame);


  IncrementalPlot *plot = new IncrementalPlot(this);
  this->dataPtr->plots.push_back(plot);
  this->dataPtr->plotLayout->addWidget(plot);

  this->dataPtr->labelList = new DragableListWidget(this);
  this->dataPtr->labelList->setDragEnabled(true);
  this->dataPtr->labelList->setDragDropMode(QAbstractItemView::DragOnly);
  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  item->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(item);

  QPushButton *addPlotButton = new QPushButton("Add");
  connect(addPlotButton, SIGNAL(clicked()), this, SLOT(OnAddPlot()));

  QCheckBox *pauseCheck = new QCheckBox;
  pauseCheck->setChecked(this->dataPtr->paused);
  connect(pauseCheck, SIGNAL(clicked(bool)), this, SLOT(OnPause(bool)));

  QHBoxLayout *pauseLayout = new QHBoxLayout;
  pauseLayout->addWidget(pauseCheck);
  pauseLayout->addWidget(new QLabel("Pause"));
  pauseLayout->addWidget(addPlotButton);
  pauseLayout->addStretch(1);

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addWidget(this->dataPtr->labelList);
  leftLayout->addLayout(pauseLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(leftLayout);
  mainLayout->addWidget(plotScrollArea, 2);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->sub = this->dataPtr->node->Subscribe("~/diagnostics",
      &Diagnostics::OnMsg, this);

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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update all the plots
  for (auto &plot : this->dataPtr->plots)
  {
    plot->Update();
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnMsg(ConstDiagnosticsPtr &_msg)
{
  // Make sure plotting is not paused.
  if (this->dataPtr->paused)
    return;

  // Make sure there is timing information
  if (_msg->time_size() == 0)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  common::Time wallTime, elapsedTime;

  wallTime = msgs::Convert(_msg->real_time());

  // Add real-time factor if it has been requested.
  for (auto &plot : this->dataPtr->plots)
  {
    if (plot->HasCurve(QString("Real Time Factor")))
    {
      plot->Add(QString("Real Time Factor"),
               QPointF(wallTime.Double(), _msg->real_time_factor()));
    }
  }

  // Process each time point
  for (int i = 0; i < _msg->time_size(); ++i)
  {
    QString qstr = QString::fromStdString(_msg->time(i).name());

    // Add the time label to the list if it's not already there.
    QList<QListWidgetItem*> items = this->dataPtr->labelList->findItems(qstr,
        Qt::MatchExactly);

    if (items.size() == 0)
    {
      QListWidgetItem *item = new QListWidgetItem(qstr);
      item->setToolTip(tr("Drag onto graph to plot"));
      this->dataPtr->labelList->addItem(item);
    }

    QString labelStr(_msg->time(i).name().c_str());

    // Check to see if the data belongs in a plot, and add it.
    for (auto &plot : this->dataPtr->plots)
    {
      if (plot->HasCurve(labelStr))
      {
        elapsedTime = msgs::Convert(_msg->time(i).elapsed());

        double msTime = elapsedTime.Double() * 1e3;
        QPointF pt(wallTime.Double(), msTime);

        plot->Add(labelStr, pt);
      }
    }
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnPause(bool _value)
{
  this->dataPtr->paused = _value;
}

/////////////////////////////////////////////////
void Diagnostics::OnAddPlot()
{
  IncrementalPlot *plot = new IncrementalPlot(this);
  this->dataPtr->plotLayout->addWidget(plot);
  this->dataPtr->plots.push_back(plot);
}

/////////////////////////////////////////////////
bool Diagnostics::eventFilter(QObject *o, QEvent *e)
{
  if (e->type() == QEvent::Wheel)
  {
    e->ignore();
    return true;
  }

  return QWidget::eventFilter(o, e);
}
