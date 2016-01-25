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
#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief PlotCanvas private data
    struct PlotCanvasPrivate
    {
      /// \brief Text label
      public: QLabel *title;

      /// \brief Layout that contains all the plots.
      public: QLayout *plotLayout;

      /// \brief Mutex to protect the point map
      public: std::mutex mutex;

      /// \brief Plot widgets
      public: std::map<unsigned int, IncrementalPlot *> plots;
    };
  }
}

/////////////////////////////////////////////////
PlotCanvas::PlotCanvas(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new PlotCanvasPrivate())
{
  // Plot title
  this->dataPtr->title = new QLabel("Plot Name");
  QHBoxLayout *titleLayout = new QHBoxLayout;
  titleLayout->addWidget(this->dataPtr->title);
  titleLayout->setAlignment(Qt::AlignHCenter);

  // X and Y variables
  VariablePillContainer *xVariableContainer = new VariablePillContainer(this);
  xVariableContainer->SetText("x: ");
  xVariableContainer->SetMaxSize(1);
  VariablePillContainer *yVariableContainer = new VariablePillContainer(this);
  yVariableContainer->SetText("y: ");

  connect(yVariableContainer, SIGNAL(VariableAdded(unsigned int, std::string)),
      this, SLOT(OnAddVariable(unsigned int, std::string)));
  connect(yVariableContainer, SIGNAL(VariableRemoved(unsigned int)),
      this, SLOT(OnRemoveVariable(unsigned int)));

  QVBoxLayout *variableContainerLayout = new QVBoxLayout;
  variableContainerLayout->addWidget(xVariableContainer);
  variableContainerLayout->addWidget(yVariableContainer);

  // plot
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
  this->dataPtr->plotLayout = new QVBoxLayout;
  plotFrame->setLayout(this->dataPtr->plotLayout);

  plotScrollArea->setWidget(plotFrame);

  // empty plot
  IncrementalPlot *plot = new IncrementalPlot(this);
  this->dataPtr->plotLayout->addWidget(plot);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(titleLayout);
  mainLayout->addLayout(variableContainerLayout);
  mainLayout->addWidget(plotScrollArea);
  this->setLayout(mainLayout);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(60);
}

/////////////////////////////////////////////////
PlotCanvas::~PlotCanvas()
{
}

/////////////////////////////////////////////////
void PlotCanvas::OnAddVariable(const unsigned int _id,
    const std::string &_variable)
{
  IncrementalPlot *plot = new IncrementalPlot(this);
  this->dataPtr->plotLayout->addWidget(plot);
  plot->AddCurve(QString(_variable.c_str()));
  this->dataPtr->plots[_id] = plot;
}

/////////////////////////////////////////////////
void PlotCanvas::OnRemoveVariable(const unsigned int _id)
{
  auto it = this->dataPtr->plots.find(_id);
  if (it == this->dataPtr->plots.end())
    return;

  this->dataPtr->plotLayout->takeAt(it->first);
  delete it->second;
  this->dataPtr->plots.erase(it);
}

/////////////////////////////////////////////////
void PlotCanvas::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update all the plots
  for (const auto plot : this->dataPtr->plots)
  {
    plot.second->Update();
  }
}
