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

#include <mutex>

#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotWindow.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the PlotWindow class
    class PlotWindowPrivate
    {
      /// \brief The list of variable labels.
      public: QListWidget *labelList;

      /// \brief True when plotting is paused.
      public: bool paused = false;

      /// \brief Action to pause plotting
      public: QAction *plotPlayAct;

      /// \brief Action to resume plotting
      public: QAction *plotPauseAct;

      /// \brief Layout to hold all the canvases.
      public: QVBoxLayout *canvasLayout;

      /// \brief Mutex to protect the canvas updates
      public: std::mutex mutex;
    };
  }
}

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

  protected: virtual Qt::DropActions supportedDropActions() const
             {
               return Qt::LinkAction;
             }
};

/////////////////////////////////////////////////
PlotWindow::PlotWindow(QWidget *_parent)
  : QDialog(_parent),
    dataPtr(new PlotWindowPrivate())
{
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle("Gazebo: Plotting Utility");
  this->setObjectName("plotWindow");
  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowCloseButtonHint | Qt::WindowStaysOnTopHint |
      Qt::CustomizeWindowHint);

  // new empty canvas
  this->dataPtr->canvasLayout = new QVBoxLayout;
  this->AddCanvas();

  // add button
  QPushButton *addCanvasButton = new QPushButton("+");
  addCanvasButton->setObjectName("plotAddCanvas");
  QGraphicsDropShadowEffect *addCanvasShadow = new QGraphicsDropShadowEffect();
  addCanvasShadow->setBlurRadius(8);
  addCanvasShadow->setOffset(0, 0);
  addCanvasButton->setGraphicsEffect(addCanvasShadow);
  //addCanvasButton->setFixedSize(QSize(25, 25));
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

  this->dataPtr->plotPlayAct = new QAction(QIcon(":/images/play_dark.png"),
      tr("Play"), this);
  this->dataPtr->plotPlayAct->setStatusTip(tr("Continue Plotting"));
  this->dataPtr->plotPlayAct->setVisible(false);
  connect(this->dataPtr->plotPlayAct, SIGNAL(triggered()),
      this, SLOT(OnPlay()));

  this->dataPtr->plotPauseAct = new QAction(QIcon(":/images/pause_dark.png"),
      tr("Pause"), this);
  this->dataPtr->plotPauseAct->setStatusTip(tr("Pause Plotting"));
  this->dataPtr->plotPauseAct->setVisible(true);
  connect(this->dataPtr->plotPauseAct, SIGNAL(triggered()),
      this, SLOT(OnPause()));

  QHBoxLayout *bottomPanelLayout = new QHBoxLayout;
  QToolBar *playToolbar = new QToolBar;
  playToolbar->setObjectName("plotToolbar");
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

  QVBoxLayout *leftLayout = new QVBoxLayout;
  leftLayout->addWidget(this->dataPtr->labelList);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addLayout(leftLayout);
  mainLayout->addLayout(plotLayout);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(30);

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
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow()
{
  this->dataPtr->paused = true;
  this->Clear();
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
  connect(canvas, SIGNAL(CanvasDeleted()), this, SLOT(OnRemoveCanvas()));

  this->dataPtr->canvasLayout->addWidget(canvas);
  return canvas;
}

/////////////////////////////////////////////////
void PlotWindow::RemoveCanvas(PlotCanvas *_canvas)
{
  int idx = this->dataPtr->canvasLayout->indexOf(_canvas);
  if (idx < 0)
    return;

  this->dataPtr->canvasLayout->takeAt(idx);
  _canvas->deleteLater();
}

/////////////////////////////////////////////////
void PlotWindow::Clear()
{
  while (this->CanvasCount() > 0u)
  {
    QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(0);
    PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
    this->RemoveCanvas(canvas);
  }
}

/////////////////////////////////////////////////
unsigned int PlotWindow::CanvasCount() const
{
  return static_cast<unsigned int>(this->dataPtr->canvasLayout->count());
}

/////////////////////////////////////////////////
void PlotWindow::OnAddCanvas()
{
  this->AddCanvas();
}

/////////////////////////////////////////////////
void PlotWindow::OnRemoveCanvas()
{
  PlotCanvas *canvas = qobject_cast<PlotCanvas *>(QObject::sender());
  if (!canvas)
    return;

  this->RemoveCanvas(canvas);

  // add an empty canvas if the plot window is now empty
  if (this->dataPtr->canvasLayout->isEmpty())
    this->AddCanvas();
}

/////////////////////////////////////////////////
void PlotWindow::Update()
{
  if (this->dataPtr->paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
  {
    QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(i);
    PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
    if (!canvas)
      continue;
    canvas->Update();
  }
}
