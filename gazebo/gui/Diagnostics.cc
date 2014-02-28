/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/IncrementalPlot.hh"
#include "gazebo/gui/Diagnostics.hh"

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
  : QDialog(_parent)
{
  this->paused = false;
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Diagnostics");
  this->setObjectName("diagnosticsPlot");

  this->plotLayout = new QVBoxLayout;

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
  plotFrame->setLayout(this->plotLayout);

  plotScrollArea->setWidget(plotFrame);


  IncrementalPlot *plot = new IncrementalPlot(this);
  this->plots.push_back(plot);
  this->plotLayout->addWidget(plot);

  this->labelList = new DragableListWidget(this);
  this->labelList->setDragEnabled(true);
  this->labelList->setDragDropMode(QAbstractItemView::DragOnly);
  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  item->setToolTip(tr("Drag onto graph to plot"));
  this->labelList->addItem(item);

  QPushButton *addPlotButton = new QPushButton("Add");
  connect(addPlotButton, SIGNAL(clicked()), this, SLOT(OnAddPlot()));

  QCheckBox *pauseCheck = new QCheckBox;
  pauseCheck->setChecked(this->paused);
  connect(pauseCheck, SIGNAL(clicked(bool)), this, SLOT(OnPause(bool)));

  QHBoxLayout *pauseLayout = new QHBoxLayout;
  pauseLayout->addWidget(pauseCheck);
  pauseLayout->addWidget(new QLabel("Pause"));
  pauseLayout->addWidget(addPlotButton);
  pauseLayout->addStretch(1);

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addWidget(this->labelList);
  leftLayout->addLayout(pauseLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(leftLayout);
  mainLayout->addWidget(plotScrollArea, 2);

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

  // Update all the plots
  for (std::vector<IncrementalPlot *>::iterator iter = this->plots.begin();
      iter != this->plots.end(); ++iter)
  {
    (*iter)->Update();
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnMsg(ConstDiagnosticsPtr &_msg)
{
  // Make sure plotting is not paused.
  if (this->paused)
    return;

  // Make sure there is timing information
  if (_msg->time_size() == 0)
    return;

  boost::mutex::scoped_lock lock(this->mutex);

  common::Time wallTime, elapsedTime;

  wallTime = msgs::Convert(_msg->real_time());

  // Add real-time factor if it has been requested.
  for (std::vector<IncrementalPlot*>::iterator iter = this->plots.begin();
      iter != this->plots.end(); ++iter)
  {
    if ((*iter)->HasCurve(QString("Real Time Factor")))
    {
      (*iter)->Add(QString("Real Time Factor"),
                   QPointF(wallTime.Double(), _msg->real_time_factor()));
    }
  }

  // Process each time point
  for (int i = 0; i < _msg->time_size(); ++i)
  {
    QString qstr = QString::fromStdString(_msg->time(i).name());

    // Add the time label to the list if it's not already there.
    QList<QListWidgetItem*> items = this->labelList->findItems(qstr,
        Qt::MatchExactly);

    if (items.size() == 0)
    {
      QListWidgetItem *item = new QListWidgetItem(qstr);
      item->setToolTip(tr("Drag onto graph to plot"));
      this->labelList->addItem(item);
    }

    QString labelStr(_msg->time(i).name().c_str());

    // Check to see if the data belongs in a plot, and add it.
    for (std::vector<IncrementalPlot*>::iterator iter = this->plots.begin();
        iter != this->plots.end(); ++iter)
    {
      if ((*iter)->HasCurve(labelStr))
      {
        elapsedTime = msgs::Convert(_msg->time(i).elapsed());

        double msTime = elapsedTime.Double() * 1e3;
        QPointF pt(wallTime.Double(), msTime);

        (*iter)->Add(labelStr, pt);
      }
    }
  }
}

/////////////////////////////////////////////////
void Diagnostics::OnPause(bool _value)
{
  this->paused = _value;
}

/////////////////////////////////////////////////
void Diagnostics::OnAddPlot()
{
  IncrementalPlot *plot = new IncrementalPlot(this);
  this->plotLayout->addWidget(plot);
  this->plots.push_back(plot);
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
