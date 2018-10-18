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

#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/Palette.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotManager.hh"
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

      /// \brief Flag to indicate whether the plots should be restarted.
      public: bool restart = false;
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
  this->dataPtr->canvasLayout->setSpacing(20);
  this->AddCanvas();

  // add button
  QPushButton *addCanvasButton = new QPushButton("+");
  addCanvasButton->setObjectName("plotAddCanvas");
  addCanvasButton->setDefault(false);
  addCanvasButton->setAutoDefault(false);
  addCanvasButton->setToolTip("Add a new canvas");
  QGraphicsDropShadowEffect *addCanvasShadow = new QGraphicsDropShadowEffect();
  addCanvasShadow->setBlurRadius(8);
  addCanvasShadow->setOffset(0, 0);
  addCanvasButton->setGraphicsEffect(addCanvasShadow);
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
  this->dataPtr->plotPlayAct->setToolTip(tr("Continue plotting"));
  this->dataPtr->plotPlayAct->setVisible(false);
  connect(this->dataPtr->plotPlayAct, SIGNAL(triggered()),
      this, SLOT(OnPlay()));

  this->dataPtr->plotPauseAct = new QAction(QIcon(":/images/pause_dark.png"),
      tr("Pause"), this);
  this->dataPtr->plotPauseAct->setToolTip(
      tr("Pause plotting (not simulation)"));
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

  // Plot layout
  QVBoxLayout *plotLayout = new QVBoxLayout;
  plotLayout->addLayout(this->dataPtr->canvasLayout);
  plotLayout->addLayout(addButtonLayout);
  plotLayout->addWidget(bottomFrame);
  plotLayout->setStretchFactor(this->dataPtr->canvasLayout, 1);
  plotLayout->setStretchFactor(addButtonLayout, 0);
  plotLayout->setStretchFactor(bottomFrame, 0);

  auto plotFrame = new QFrame;
  plotFrame->setLayout(plotLayout);

  // Palette
  auto plotPalette = new Palette(this);

  auto splitter = new QSplitter(Qt::Horizontal, this);
  splitter->addWidget(plotPalette);
  splitter->addWidget(plotFrame);
  splitter->setCollapsible(0, true);
  splitter->setCollapsible(1, false);

  QList<int> sizes;
  sizes << 30 << 70;
  splitter->setSizes(sizes);

  auto mainLayout = new QHBoxLayout;
  mainLayout->addWidget(splitter);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->setSizeGripEnabled(true);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(30);

  PlotManager::Instance()->AddWindow(this);

  this->setMinimumSize(640, 480);
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow()
{
  this->dataPtr->paused = true;
  PlotManager::Instance()->RemoveWindow(this);
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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->restart)
  {
    for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
    {
      QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(i);
      PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
      if (!canvas)
        continue;
      canvas->Restart();
    }
    this->dataPtr->restart = false;
  }

  if (this->dataPtr->paused)
    return;

  for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
  {
    QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(i);
    PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
    if (!canvas)
      continue;
    canvas->Update();
  }
}

/////////////////////////////////////////////////
void PlotWindow::Restart()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->restart = true;
}

/////////////////////////////////////////////////
void PlotWindow::Export()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
  {
    QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(i);
    PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
    if (!canvas)
      continue;

    for (const auto &plot : canvas->Plots())
    {
      for (const auto &curve : plot->Curves())
      {
        auto c = curve.lock();
        if (!c)
          continue;

        for (unsigned int j = 0; j < c->Size(); ++j)
        {
          ignition::math::Vector2d pt = c->Point(j);
          std::cerr << pt.X() << ", " << pt.Y() << std::endl;
        }
      }
    }
  }
}
