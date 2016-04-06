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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/ExportDialog.hh"
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
  : QWidget(_parent),
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

  // export button
  QPushButton *exportPlotButton = new QPushButton("Export");
  exportPlotButton->setIcon(QIcon(":/images/file_upload.svg"));
  exportPlotButton->setObjectName("plotExport");
  exportPlotButton->setDefault(false);
  exportPlotButton->setAutoDefault(false);
  exportPlotButton->setToolTip("Export plot data");
  QGraphicsDropShadowEffect *exportPlotShadow = new QGraphicsDropShadowEffect();
  exportPlotShadow->setBlurRadius(8);
  exportPlotShadow->setOffset(0, 0);
  exportPlotButton->setGraphicsEffect(exportPlotShadow);
  connect(exportPlotButton, SIGNAL(clicked()), this, SLOT(OnExport()));


  QHBoxLayout *addButtonLayout = new QHBoxLayout;
  addButtonLayout->addWidget(exportPlotButton);
  addButtonLayout->addStretch();
  addButtonLayout->addWidget(addCanvasButton);
  addButtonLayout->setAlignment(Qt::AlignRight | Qt::AlignBottom);
  addButtonLayout->setContentsMargins(0, 0, 0, 0);
  addCanvasButton->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  // main layout
  QVBoxLayout *plotLayout = new QVBoxLayout;
  plotLayout->addLayout(this->dataPtr->canvasLayout);
  plotLayout->addLayout(addButtonLayout);
  //plotLayout->addWidget(bottomFrame);
  plotLayout->setStretchFactor(this->dataPtr->canvasLayout, 1);
  plotLayout->setStretchFactor(addButtonLayout, 0);
  //plotLayout->setStretchFactor(bottomFrame, 0);

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

  QShortcut *space = new QShortcut(Qt::Key_Space, this);
  QObject::connect(space, SIGNAL(activated()), this, SLOT(TogglePause()));

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(30);

  PlotManager::Instance()->AddWindow(this);

  this->setMinimumSize(640, 480);
}

/////////////////////////////////////////////////
PlotWindow::~PlotWindow()
{
  PlotManager::Instance()->RemoveWindow(this);
  this->Clear();
}

/////////////////////////////////////////////////
PlotCanvas *PlotWindow::AddCanvas()
{
  PlotCanvas *canvas = new PlotCanvas(this);
  connect(canvas, SIGNAL(CanvasDeleted()), this, SLOT(OnRemoveCanvas()));

  this->dataPtr->canvasLayout->addWidget(canvas);

  this->UpdateCanvas();

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
  else
  {
    this->UpdateCanvas();
  }
}

/////////////////////////////////////////////////
void PlotWindow::UpdateCanvas()
{
  // disable Delete Canvas option in settings if there is only one
  // canvas in the window
  QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(0);
  if (item)
  {
    PlotCanvas *plotCanvas = qobject_cast<PlotCanvas *>(item->widget());
    if (plotCanvas)
    {
      plotCanvas->SetDeleteCanvasEnabled(
          this->dataPtr->canvasLayout->count() != 1);
    }
  }
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
void PlotWindow::TogglePause()
{
  MainWindow *mainWindow = gui::get_main_window();
  if (!mainWindow)
    return;

  if (mainWindow->IsPaused())
    mainWindow->Play();
  else
    mainWindow->Pause();
}

/////////////////////////////////////////////////
void PlotWindow::OnExport()
{
  // Get the plots that have data.
  std::list<PlotCanvas*> plots;
  for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
  {
    bool hasData = false;
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

        hasData = hasData || c->Size() > 0;
      }
    }

    if (hasData)
      plots.push_back(canvas);
  }

  // Display an error message if no plots have data.
  if (plots.empty())
  {
    QMessageBox msgBox(
        QMessageBox::Information,
        QString("Unable to export"),
        QString(
          "No data to export.\nAdd variables with data to a graph first."),
        QMessageBox::Close,
        this,
        Qt::Window | Qt::WindowTitleHint |
        Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
    msgBox.exec();
  }
  else
  {
    ExportDialog *dialog = new ExportDialog(this, plots);
    dialog->show();
  }
}

/////////////////////////////////////////////////
std::list<PlotCanvas*> PlotWindow::Plots()
{
  std::list<PlotCanvas*> plots;

  for (int i = 0; i < this->dataPtr->canvasLayout->count(); ++i)
  {
    QLayoutItem *item = this->dataPtr->canvasLayout->itemAt(i);
    PlotCanvas *canvas = qobject_cast<PlotCanvas *>(item->widget());
    if (!canvas)
      continue;
    plots.push_back(canvas);
  }

  return plots;
}
