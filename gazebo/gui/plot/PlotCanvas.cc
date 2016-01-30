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

#include <set>
#include <mutex>

#include "gazebo/common/Console.hh"

#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/VariablePill.hh"
#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    struct PlotCanvasData
    {
      /// brief Poniter to the plot
      public: IncrementalPlot *plot;

      /// \brief A map of variable ids to their curve ids in the plot.
      public: std::map<unsigned int, unsigned int> variables;
    };

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
      // public: std::map<unsigned int, IncrementalPlot *> plots;

      /// \brief Plot canvas data;
      public: std::vector<PlotCanvasData *> plotCanvasData;

      /// \brief Pointer to an empty plot.
      public: IncrementalPlot *emptyPlot = NULL;
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
  xVariableContainer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  VariablePillContainer *yVariableContainer = new VariablePillContainer(this);
  yVariableContainer->SetText("y: ");
  yVariableContainer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);

  connect(yVariableContainer,
      SIGNAL(VariableAdded(unsigned int, unsigned int, std::string)),
      this, SLOT(OnAddVariable(unsigned int, unsigned int, std::string)));
  connect(yVariableContainer,
      SIGNAL(VariableRemoved(unsigned int, unsigned int)),
      this, SLOT(OnRemoveVariable(unsigned int, unsigned int)));
  connect(yVariableContainer, SIGNAL(VariableMoved(unsigned int, unsigned int)),
      this, SLOT(OnMoveVariable(unsigned int, unsigned int)));

  QVBoxLayout *variableContainerLayout = new QVBoxLayout;
  variableContainerLayout->addWidget(xVariableContainer);
  variableContainerLayout->addWidget(yVariableContainer);


  // plot
  QScrollArea *plotScrollArea = new QScrollArea(this);
  plotScrollArea->setLineWidth(0);
  plotScrollArea->setFrameShape(QFrame::NoFrame);
  plotScrollArea->setFrameShadow(QFrame::Plain);
  plotScrollArea->setSizePolicy(QSizePolicy::Minimum,
                                QSizePolicy::Expanding);

  plotScrollArea->setWidgetResizable(true);
  plotScrollArea->viewport()->installEventFilter(this);

  QFrame *plotFrame = new QFrame(plotScrollArea);
  plotFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
  this->dataPtr->plotLayout = new QVBoxLayout;
  plotFrame->setLayout(this->dataPtr->plotLayout);

  plotScrollArea->setWidget(plotFrame);

  // empty plot
  this->dataPtr->emptyPlot = new IncrementalPlot(this);
  this->dataPtr->plotLayout->addWidget(this->dataPtr->emptyPlot);

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
    const unsigned int _targetId, const std::string &_variable)
{
  std::cerr << " PlotCanvas:: on add variable " << _variable << " " << _id << std::endl;

  if (_targetId != VariablePill::EMPTY_ID)
  {
    // Add a variable to existing plot
    for (auto it = this->dataPtr->plotCanvasData.begin();
        it != this->dataPtr->plotCanvasData.end(); ++it)
    {
      auto v = (*it)->variables.find(_targetId);
      if (v != (*it)->variables.end())
      {
        PlotCurve *curve = (*it)->plot->AddCurve(_variable);
        (*it)->variables[_id] = curve->id;
        break;
      }
    }
  }
  else
  {
    // create new plot for the variable and add plot to canvas
    IncrementalPlot *plot = new IncrementalPlot(this);
    plot->setAutoDelete(false);
    this->dataPtr->plotLayout->addWidget(plot);

    PlotCurve *curve = plot->AddCurve(_variable);
    PlotCanvasData *plotData = new PlotCanvasData;
    plotData->plot = plot;
    plotData->variables[_id] = curve->id;
    this->dataPtr->plotCanvasData.push_back(plotData);
    std::cerr << " create new plot " << std::endl;
  }

  // hide initial empty plot
  if (!this->dataPtr->plotCanvasData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(false);


  // TODO remove me later
  this->debug();
}

/////////////////////////////////////////////////
void PlotCanvas::OnRemoveVariable(const unsigned int _id,
    const unsigned int /*_targetId*/)
{
  std::cerr << " PlotCanvas:: on remove variable " << _id << std::endl;

  // new logic
  for (auto it = this->dataPtr->plotCanvasData.begin();
      it != this->dataPtr->plotCanvasData.end(); ++it)
  {
    auto v = (*it)->variables.find(_id);
    if (v != (*it)->variables.end())
    {
      unsigned int curveId = v->second;
      (*it)->variables.erase(v);

      // delete whole plot if no more curves
      if ((*it)->variables.empty())
      {
        this->dataPtr->plotLayout->takeAt(
            this->dataPtr->plotLayout->indexOf((*it)->plot));
        (*it)->plot->RemoveCurve(curveId);
        delete (*it)->plot;
        delete (*it);
        this->dataPtr->plotCanvasData.erase(it);
      }
      else
      {
        // TODO remove / detach curve from plot
        (*it)->plot->RemoveCurve(curveId);
      }
      break;
    }
  }

  if (this->dataPtr->plotCanvasData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(true);
}

/////////////////////////////////////////////////
void PlotCanvas::OnMoveVariable(const unsigned int _id,
    const unsigned int _targetId)
{
  std::cerr << " PlotCanvas:: on move variable " << std::endl;
  auto plotIt = this->dataPtr->plotCanvasData.end();
  auto targetPlotIt = this->dataPtr->plotCanvasData.end();
  unsigned int curveId = 0;

  // find plot which the variable belongs to
  // find target plot (if any) that the variable will be moved to
  for (auto it = this->dataPtr->plotCanvasData.begin();
      it != this->dataPtr->plotCanvasData.end(); ++it)
  {
    auto v = (*it)->variables.find(_id);
    if (v != (*it)->variables.end())
    {
      plotIt = it;
      curveId = v->second;
    }

    if ((*it)->variables.find(_targetId) != (*it)->variables.end())
      targetPlotIt = it;

    if (plotIt != this->dataPtr->plotCanvasData.end() &&
        targetPlotIt != this->dataPtr->plotCanvasData.end())
      break;
  }

  // detach from old plot and attach to new one
  if (plotIt != this->dataPtr->plotCanvasData.end())
  {
    std::cerr << " move variable!! " <<  std::endl;

    PlotCanvasData *plotCanvasData = *plotIt;

    // detach variable from plot (qwt plot doesn't seem to do anything
    // apart from setting the plot item to null)
    PlotCurve *plotCurve = plotCanvasData->plot->DetachCurve(curveId);
    plotCanvasData->variables.erase(plotCanvasData->variables.find(_id));

    if (targetPlotIt != this->dataPtr->plotCanvasData.end())
    {
      // attach variable to target plot
      (*targetPlotIt)->plot->AttachCurve(plotCurve);
      (*targetPlotIt)->variables[_id] = plotCurve->id;
    }
    else
    {
      std::cerr << " move to new plot " << std::endl;
      // add variable to new plot
      this->OnAddVariable(_id, VariablePill::EMPTY_ID, plotCurve->label);

    }
    // delete plot if empty
    if (plotCanvasData->variables.empty())
    {
      std::cerr << " delete plot as it's now empty " << std::endl;
      this->dataPtr->plotLayout->takeAt(
          this->dataPtr->plotLayout->indexOf(plotCanvasData->plot));

      // careful about deleting by iterator (plotIt) as it may have been
      // changed if a new plot is added to the vector in the OnAddVariable call
      // in this function
      for (auto it = this->dataPtr->plotCanvasData.begin();
          it != this->dataPtr->plotCanvasData.end(); ++it)
      {
        if ((*it) == plotCanvasData)
        {
          this->dataPtr->plotCanvasData.erase(it);
          break;
        }
      }
        /*this->dataPtr->plotCanvasData.erase(std::remove(
          this->dataPtr->plotCanvasData.begin(),
          this->dataPtr->plotCanvasData.end(), plotCanvasData),
          this->dataPtr->plotCanvasData.end());*/

      //plotCanvasData->plot->hide();
      plotCanvasData->plot->detachItems(QwtPlotItem::Rtti_PlotItem, false);
      delete plotCanvasData->plot;
      delete plotCanvasData;


      // this->dataPtr->plotCanvasData.erase(plotIt);
    }
  }

  // TODO remove me later
  this->debug();
}

/////////////////////////////////////////////////
void PlotCanvas::Update()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Update all the plots
  for (auto &p : this->dataPtr->plotCanvasData)
    p->plot->Update();
}

/////////////////////////////////////////////////
bool PlotCanvas::eventFilter(QObject *_o, QEvent *_e)
{
  if (_e->type() == QEvent::Wheel)
  {
    _e->ignore();
    return true;
  }

  return QWidget::eventFilter(_o, _e);
}

/////////////////////////////////////////////////
void PlotCanvas::debug()
{
  std::cerr << "================" << std::endl;
  for (auto it = this->dataPtr->plotCanvasData.begin();
      it != this->dataPtr->plotCanvasData.end(); ++it)
  {
    for (auto &v : (*it)->variables)
    {
      std::cerr << v.first << " : " << v.second << std::endl;
    }
    std::cerr << "-------------------" << std::endl;
  }
  std::cerr << "================" << std::endl;
}
