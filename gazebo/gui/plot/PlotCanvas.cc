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
    struct PlotData
    {
      /// \brief Unique id of the plot
      public: unsigned int id;

      /// brief Poniter to the plot
      public: IncrementalPlot *plot = NULL;

      /// \brief A map of container variable ids to their plot curve ids.
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
      public: std::map<unsigned int, PlotData *> plotData;

      /// \brief Pointer to an empty plot.
      public: IncrementalPlot *emptyPlot = NULL;

      /// \brief Container for all the variables on the Y axis.
      public: VariablePillContainer *yVariableContainer = NULL;

      /// \brief Global plot counter.
      public: static unsigned int globalPlotId;
    };
  }
}

unsigned int PlotCanvasPrivate::globalPlotId = 0;

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

  // Settings
  QMenu *settingsMenu = new QMenu;
/*  QWidgetAction *settingsAction = new QWidgetAction(settingsMenu);
  settingsAction->setDefaultWidget(settingsWidget);
  settingsMenu->addAction(settingsAction);*/

  QAction *clearPlotAct = new QAction("Clear all fields", settingsMenu);
  clearPlotAct->setStatusTip(tr("Clear variables and all plots on canvas"));
  connect(clearPlotAct, SIGNAL(triggered()), this, SLOT(OnClearCanvas()));
  QAction *deletePlotAct = new QAction("Delete Plot", settingsMenu);
  deletePlotAct->setStatusTip(tr("Delete entire canvas"));
  connect(deletePlotAct, SIGNAL(triggered()), this, SLOT(OnDeleteCanvas()));

  settingsMenu->addAction(clearPlotAct);
  settingsMenu->addAction(deletePlotAct);

  QToolButton *settingsButton = new QToolButton();
  settingsButton->installEventFilter(this);
//  settingsButton->setFixedSize(QSize(35, 35));
//  settingsButton->setIconSize(QSize(25, 25));
  settingsButton->setToolTip(tr("Settings"));
  settingsButton->setIcon(QIcon(":/images/settings.png"));
  settingsButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  settingsButton->setPopupMode(QToolButton::InstantPopup);
  settingsButton->setMenu(settingsMenu);
/*  settingsButton->setStyleSheet("\
      QToolButton {\
        margin: 0px;\
        padding: 0px;\
      }\
      QToolButton::menu-indicator {\
        image: none;\
      }\
      QToolButton:hover, QToolButton:pressed {\
        background-color: #d47402;\
        border: none;\
      }");*/

  QHBoxLayout *settingsLayout = new QHBoxLayout;
  settingsLayout->addWidget(settingsButton);

  QHBoxLayout *titleSettingsLayout = new QHBoxLayout;
  titleSettingsLayout->addLayout(titleLayout);
  titleSettingsLayout->addLayout(settingsLayout);

  // X and Y variables
  VariablePillContainer *xVariableContainer = new VariablePillContainer(this);
  xVariableContainer->SetText("x: ");
  xVariableContainer->SetMaxSize(1);
  xVariableContainer->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  this->dataPtr->yVariableContainer = new VariablePillContainer(this);
  this->dataPtr->yVariableContainer->SetText("y: ");
  this->dataPtr->yVariableContainer->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Fixed);

  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableAdded(unsigned int, std::string, unsigned int)),
      this, SLOT(OnAddVariable(unsigned int, std::string, unsigned int)));
  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableRemoved(unsigned int, unsigned int)),
      this, SLOT(OnRemoveVariable(unsigned int, unsigned int)));
  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableMoved(unsigned int, unsigned int)),
      this, SLOT(OnMoveVariable(unsigned int, unsigned int)));

  QVBoxLayout *variableContainerLayout = new QVBoxLayout;
  variableContainerLayout->addWidget(xVariableContainer);
  variableContainerLayout->addWidget(this->dataPtr->yVariableContainer);


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
  mainLayout->addLayout(titleSettingsLayout);
  mainLayout->addLayout(variableContainerLayout);
  mainLayout->addWidget(plotScrollArea);
  this->setLayout(mainLayout);

  QTimer *displayTimer = new QTimer(this);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(Update()));
  displayTimer->start(30);
}

/////////////////////////////////////////////////
PlotCanvas::~PlotCanvas()
{
  this->Clear();
}

/////////////////////////////////////////////////
unsigned int PlotCanvas::AddVariable(const std::string &_variable)
{
  // add to container and let the signals/slots do the work on adding the
  // a new plot with the curve in the overloaded AddVariable function
  return this->dataPtr->yVariableContainer->AddVariablePill(_variable);
}

/////////////////////////////////////////////////
void PlotCanvas::AddVariable(const unsigned int _id,
    const std::string &_variable, const unsigned int _plotId)
{
  auto it = this->dataPtr->plotData.find(_plotId);
  if (it != this->dataPtr->plotData.end())
  {
    PlotData *plotData = it->second;
    PlotCurve *curve = plotData->plot->AddCurve(_variable);
    plotData->variables[_id] = curve->id;
  }
  else
  {
    return;
  }

  // hide initial empty plot
  if (!this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(false);

  // TODO remove me later
  this->debug();
}

/////////////////////////////////////////////////
void PlotCanvas::AddVariable(const unsigned int _id,
    const std::string &_variable)
{
  // create new plot for the variable and add plot to canvas
  IncrementalPlot *plot = new IncrementalPlot(this);
  plot->setAutoDelete(false);
  this->dataPtr->plotLayout->addWidget(plot);

  PlotCurve *curve = plot->AddCurve(_variable);
  PlotData *plotData = new PlotData;
  plotData->id = this->dataPtr->globalPlotId++;
  plotData->plot = plot;
  plotData->variables[_id] = curve->id;
  this->dataPtr->plotData[plotData->id] = plotData;
  std::cerr << " create new plot " << std::endl;

  // hide initial empty plot
  if (!this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(false);
}

/////////////////////////////////////////////////
void PlotCanvas::RemoveVariable(const unsigned int _id)
{
  // loop through plots and find the variable to be removed
  for (auto it = this->dataPtr->plotData.begin();
      it != this->dataPtr->plotData.end(); ++it)
  {
    auto v = it->second->variables.find(_id);
    if (v != it->second->variables.end())
    {
      unsigned int curveId = v->second;
      it->second->variables.erase(v);

      // delete whole plot if no more curves
      if (it->second->variables.empty())
      {
        this->dataPtr->plotLayout->takeAt(
            this->dataPtr->plotLayout->indexOf(it->second->plot));
        it->second->plot->RemoveCurve(curveId);
        delete it->second->plot;
        delete it->second;
        this->dataPtr->plotData.erase(it);
      }
      else
      {
        // TODO remove / detach curve from plot
        it->second->plot->RemoveCurve(curveId);
      }
      break;
    }
  }

  if (this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(true);

  // remove from variable pill container
  this->dataPtr->yVariableContainer->RemoveVariablePill(_id);
}

/////////////////////////////////////////////////
void PlotCanvas::RemoveVariable(const unsigned int _id,
    const unsigned int _plotId)
{
  auto it = this->dataPtr->plotData.find(_plotId);
  if (it == this->dataPtr->plotData.end())
    return;

  auto v = it->second->variables.find(_id);
  if (v == it->second->variables.end())
    return;

  unsigned int curveId = v->second;
  it->second->variables.erase(v);

  // delete whole plot if no more curves
  if (it->second->variables.empty())
  {
    this->dataPtr->plotLayout->takeAt(
        this->dataPtr->plotLayout->indexOf(it->second->plot));
    it->second->plot->RemoveCurve(curveId);
    delete it->second->plot;
    delete it->second;
    this->dataPtr->plotData.erase(it);
  }
  else
  {
    // TODO remove / detach curve from plot
    it->second->plot->RemoveCurve(curveId);
  }

  if (this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
    this->dataPtr->emptyPlot->setVisible(true);

  // remove from variable pill container
  this->dataPtr->yVariableContainer->RemoveVariablePill(_id);
}

/////////////////////////////////////////////////
void PlotCanvas::RemovePlot(const unsigned int _id)
{
  auto it = this->dataPtr->plotData.find(_id);
  if (it == this->dataPtr->plotData.end())
    return;

  unsigned int plotId = it->first;

  // remove all variables except last one
  while (it->second->variables.size() > 1)
  {
    auto v = it->second->variables.begin();
    this->RemoveVariable(v->first, plotId);
  }

  // remove last variable - this will also delete the plot
  this->RemoveVariable(it->second->variables.begin()->first, plotId);
}

/////////////////////////////////////////////////
void PlotCanvas::Clear()
{
  while (!this->dataPtr->plotData.empty())
  {
    auto p = this->dataPtr->plotData.begin();
    this->RemovePlot(p->first);
  }
}

/////////////////////////////////////////////////
unsigned int PlotCanvas::VariablePlot(const unsigned int _variableId) const
{
  for (const auto it : this->dataPtr->plotData)
  {
    const auto v = it.second->variables.find(_variableId);
    if (v != it.second->variables.end())
    {
      return it.first;
    }
  }
  return EMPTY_PLOT;
}

/////////////////////////////////////////////////
void PlotCanvas::OnAddVariable(const unsigned int _id,
    const std::string &_variable, const unsigned int _targetId)
{
  std::cerr << " PlotCanvas:: on add variable " << _variable << " " << _id << std::endl;

  if (_targetId != VariablePill::EMPTY_VARIABLE)
  {
    // Add a variable to existing plot
    for (const auto it : this->dataPtr->plotData)
    {
      const auto v = it.second->variables.find(_targetId);
      if (v != it.second->variables.end())
      {
        this->AddVariable(_id, _variable, it.second->id);
        break;
      }
    }
  }
  else
  {
    // add variable to new plot
    this->AddVariable(_id, _variable);
  }
}

/////////////////////////////////////////////////
void PlotCanvas::OnRemoveVariable(const unsigned int _id,
    const unsigned int /*_targetId*/)
{
  std::cerr << " PlotCanvas:: on remove variable " << _id << std::endl;

  this->RemoveVariable(_id);
}

/////////////////////////////////////////////////
void PlotCanvas::OnMoveVariable(const unsigned int _id,
    const unsigned int _targetId)
{
  std::cerr << " PlotCanvas:: on move variable " << std::endl;
  auto plotIt = this->dataPtr->plotData.end();
  auto targetPlotIt = this->dataPtr->plotData.end();
  unsigned int curveId = 0;

  // find plot which the variable belongs to
  // find target plot (if any) that the variable will be moved to
  for (auto it = this->dataPtr->plotData.begin();
      it != this->dataPtr->plotData.end(); ++it)
  {
    auto v = it->second->variables.find(_id);
    if (v != it->second->variables.end())
    {
      plotIt = it;
      curveId = v->second;
    }

    if (it->second->variables.find(_targetId) != it->second->variables.end())
      targetPlotIt = it;

    if (plotIt != this->dataPtr->plotData.end() &&
        targetPlotIt != this->dataPtr->plotData.end())
      break;
  }

  // detach from old plot and attach to new one
  if (plotIt != this->dataPtr->plotData.end())
  {
    std::cerr << " move variable!! " <<  std::endl;

    PlotData *plotData = plotIt->second;

    // detach variable from plot (qwt plot doesn't seem to do anything
    // apart from setting the plot item to null)
    PlotCurve *plotCurve = plotData->plot->DetachCurve(curveId);
    plotData->variables.erase(plotData->variables.find(_id));

    if (targetPlotIt != this->dataPtr->plotData.end())
    {
      // attach variable to target plot
      targetPlotIt->second->plot->AttachCurve(plotCurve);
      targetPlotIt->second->variables[_id] = plotCurve->id;
    }
    else
    {
      std::cerr << " move to new plot " << std::endl;
      // add variable to new plot
      this->OnAddVariable(_id, plotCurve->label, VariablePill::EMPTY_VARIABLE);

    }
    // delete plot if empty
    if (plotData->variables.empty())
    {
      std::cerr << " delete plot as it's now empty " << std::endl;
      this->dataPtr->plotLayout->takeAt(
          this->dataPtr->plotLayout->indexOf(plotData->plot));

      // careful about deleting by iterator (plotIt) as it may have been
      // changed if a new plot is added to the vector in the OnAddVariable call
      // in this function
      for (auto it = this->dataPtr->plotData.begin();
          it != this->dataPtr->plotData.end(); ++it)
      {
        if (it->second == plotData)
        {
          this->dataPtr->plotData.erase(it);
          break;
        }
      }
        /*this->dataPtr->plotData.erase(std::remove(
          this->dataPtr->plotData.begin(),
          this->dataPtr->plotData.end(), PlotData),
          this->dataPtr->plotData.end());*/

      //plotData->plot->hide();
      plotData->plot->detachItems(QwtPlotItem::Rtti_PlotItem, false);
      delete plotData->plot;
      delete plotData;


      // this->dataPtr->plotData.erase(plotIt);
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
  for (auto p : this->dataPtr->plotData)
    p.second->plot->Update();
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
void PlotCanvas::RestartPlots()
{
  for (auto p : this->dataPtr->plotData)
  {
    for (const auto v : p.second->variables)
    {
      std::string newName = "";

      VariablePill *variable =
          this->dataPtr->yVariableContainer->GetVariablePill(v.first);
      if (!variable)
      {
        gzerr << "Unable to find variable pill with id: '" << v.first << "'."
            << std::endl;
        continue;
      }
      p.second->plot->SetCurveLabel(v.second, newName);
    }
  }
}

/////////////////////////////////////////////////
void PlotCanvas::OnClearCanvas()
{
  this->Clear();
}

/////////////////////////////////////////////////
void PlotCanvas::OnDeleteCanvas()
{
  emit CanvasDeleted();
}

/////////////////////////////////////////////////
void PlotCanvas::debug()
{
  std::cerr << "================" << std::endl;
  for (auto it = this->dataPtr->plotData.begin();
      it != this->dataPtr->plotData.end(); ++it)
  {
    for (auto &v : it->second->variables)
    {
      std::cerr << v.first << " : " << v.second << std::endl;
    }
    std::cerr << "-------------------" << std::endl;
  }
  std::cerr << "================" << std::endl;
}
