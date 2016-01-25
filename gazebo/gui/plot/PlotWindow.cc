/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotWindow.hh"
#include "gazebo/gui/plot/PlotWindowPrivate.hh"

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
PlotWindow::PlotWindow(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new PlotWindowPrivate())
{
  this->dataPtr->paused = false;
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Plotting Utility");
  this->setObjectName("PlotWindow");
  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowCloseButtonHint | Qt::WindowStaysOnTopHint |
      Qt::CustomizeWindowHint);

  // new empty canvas
  this->dataPtr->canvasLayout = new QVBoxLayout;
  this->AddCanvas();

  // add button
  QPushButton *addCanvasButton = new QPushButton("+");
  connect(addCanvasButton, SIGNAL(clicked()), this, SLOT(OnAddCanvas()));
  QVBoxLayout *addButtonLayout = new QVBoxLayout;
  addButtonLayout->addWidget(addCanvasButton);
  addButtonLayout->setAlignment(Qt::AlignRight | Qt::AlignBottom);
  addButtonLayout->setContentsMargins(0, 0, 0, 0);
  addCanvasButton->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  // Bottom toolbar
  QFrame *bottomFrame = new QFrame;
  bottomFrame->setObjectName("plotBottomFrame");
  bottomFrame->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Minimum);

  this->dataPtr->plotPlayAct = new QAction(QIcon(":/images/play.png"),
      tr("Play"), this);
  this->dataPtr->plotPlayAct->setStatusTip(tr("Continue Plotting"));
  this->dataPtr->plotPlayAct->setVisible(true);
  connect(this->dataPtr->plotPlayAct, SIGNAL(triggered()),
      this, SLOT(OnPlay()));

  this->dataPtr->plotPauseAct = new QAction(QIcon(":/images/pause.png"),
      tr("Pause"), this);
  this->dataPtr->plotPauseAct->setStatusTip(tr("Pause Plotting"));
  this->dataPtr->plotPauseAct->setVisible(false);
  connect(this->dataPtr->plotPauseAct, SIGNAL(triggered()),
      this, SLOT(OnPause()));

  QHBoxLayout *bottomPanelLayout = new QHBoxLayout;
  QToolBar *playToolbar = new QToolBar;
  playToolbar->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  playToolbar->addAction(this->dataPtr->plotPlayAct);
  playToolbar->addAction(this->dataPtr->plotPauseAct);
  bottomPanelLayout->addStretch();
  bottomPanelLayout->addWidget(playToolbar);
  bottomPanelLayout->addStretch();
  bottomPanelLayout->setContentsMargins(0, 0, 0, 0);
  bottomFrame->setLayout(bottomPanelLayout);

  // main layout
  QVBoxLayout *plotLayout = new QVBoxLayout;
  plotLayout->addLayout(this->dataPtr->canvasLayout);
  plotLayout->addLayout(addButtonLayout);
  plotLayout->addWidget(bottomFrame);
  plotLayout->setStretchFactor(this->dataPtr->canvasLayout, 1);
  plotLayout->setStretchFactor(addButtonLayout, 0);
  plotLayout->setStretchFactor(bottomFrame, 0);

  // left panel
  this->dataPtr->labelList = new DragableListWidget(this);
  this->dataPtr->labelList->setDragEnabled(true);
  this->dataPtr->labelList->setDragDropMode(QAbstractItemView::DragOnly);
  this->dataPtr->labelList->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);
  QListWidgetItem *item = new QListWidgetItem("Real Time Factor");
  item->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(item);

  //=================
  // TODO for testing - remove later
  QListWidgetItem *itema = new QListWidgetItem("Dog");
  itema->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itema);
  QListWidgetItem *itemb = new QListWidgetItem("Cat");
  itemb->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itemb);
  QListWidgetItem *itemc = new QListWidgetItem("Turtle");
  itemc->setToolTip(tr("Drag onto graph to plot"));
  this->dataPtr->labelList->addItem(itemc);

  //=================

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addWidget(this->dataPtr->labelList);
  //leftLayout->addLayout(pauseLayout);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(leftLayout);
//  mainLayout->addWidget(plotScrollArea, 2);
  mainLayout->addLayout(plotLayout);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
//  this->dataPtr->sub = this->dataPtr->node->Subscribe("~/PlotWindow",
//      &PlotWindow::OnMsg, this);

  msgs::DiagnosticControl msg;
  msg.set_enabled(true);
  this->dataPtr->node->Publish<msgs::DiagnosticControl>(
      "~/diagnostic/control", msg);
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow()
{
}

/////////////////////////////////////////////////
void PlotWindow::OnPlay()
{
  this->dataPtr->paused = false;
  this->dataPtr->plotPauseAct->setVisible(true);
  this->dataPtr->plotPlayAct->setVisible(false);
}

/////////////////////////////////////////////////
void PlotWindow::OnPause()
{
  this->dataPtr->paused = true;
  this->dataPtr->plotPauseAct->setVisible(false);
  this->dataPtr->plotPlayAct->setVisible(true);
}

/////////////////////////////////////////////////
PlotCanvas *PlotWindow::AddCanvas()
{
  PlotCanvas *canvas = new PlotCanvas(this);
  this->dataPtr->canvasLayout->addWidget(canvas);
  return canvas;
}

/////////////////////////////////////////////////
void PlotWindow::RemoveCanvas(PlotCanvas *canvas)
{
  int idx = this->dataPtr->canvasLayout->indexOf(canvas);
  if (idx < 0)
    return;

  canvas->hide();
  this->dataPtr->canvasLayout->takeAt(idx);
  delete canvas;
}

/////////////////////////////////////////////////
void PlotWindow::OnAddCanvas()
{
  this->AddCanvas();
}

/////////////////////////////////////////////////
void PlotWindow::OnRemoveCanvas()
{
  PlotCanvas *canvas =
    qobject_cast<PlotCanvas *>(QObject::sender());

  if (!canvas)
    return;

  this->RemoveCanvas(canvas);
}

/////////////////////////////////////////////////
bool PlotWindow::eventFilter(QObject *o, QEvent *e)
{
  if (e->type() == QEvent::Wheel)
  {
    e->ignore();
    return true;
  }

  return QWidget::eventFilter(o, e);
}

/////////////////////////////////////////////////
void PlotWindow::closeEvent(QCloseEvent *_evt)
{
  msgs::DiagnosticControl msg;
  msg.set_enabled(false);
  this->dataPtr->node->Publish<msgs::DiagnosticControl>(
      "~/diagnostic/control", msg);

  QDialog::closeEvent(_evt);
}



/*/////////////////////////////////////////////////
void PlotWindow::OnMsg(ConstPlotWindowPtr &_msg)
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

  // Process each variable
  for (int i = 0; i < _msg->variable_size(); ++i)
  {
    QString qstr = QString::fromStdString(_msg->variable(i).name());

    // Add the time label to the list if it's not already there.
    QList<QListWidgetItem*> items = this->dataPtr->labelList->findItems(qstr,
        Qt::MatchExactly);

    if (items.size() == 0)
    {
      QListWidgetItem *item = new QListWidgetItem(qstr);
      item->setToolTip(tr("Drag onto graph to plot"));
      this->dataPtr->labelList->addItem(item);
    }

    // Check to see if the data belongs in a plot, and add it.
    for (auto iter = this->dataPtr->plots.begin();
        iter != this->dataPtr->plots.end(); ++iter)
    {
      if ((*iter)->HasCurve(qstr))
      {
        QPointF pt(wallTime.Double(), _msg->variable(i).value());
        (*iter)->Add(qstr, pt);
      }
    }
  }

  // Process each marker
  for (int i = 0; i < _msg->marker_size(); ++i)
  {
    QString qstr = QString::fromStdString(_msg->marker(i).name());

    // Add the time label to the list if it's not already there.
    QList<QListWidgetItem*> items = this->dataPtr->labelList->findItems(qstr,
        Qt::MatchExactly);

    if (items.size() == 0)
    {
      QListWidgetItem *item = new QListWidgetItem(qstr);
      item->setToolTip(tr("Drag onto graph to plot"));
      this->dataPtr->labelList->addItem(item);
    }

    // Add all marker to all plots
    for (auto iter = this->dataPtr->plots.begin();
        iter != this->dataPtr->plots.end(); ++iter)
    {
      (*iter)->AddVLine(qstr, wallTime.Double());
    }
  }
}*/
