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

#include <map>
#include <tuple>
#include <vector>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/gui/plot/EditableLabel.hh"
#include "gazebo/gui/plot/PlotManager.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/VariablePill.hh"
#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotCanvas.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Helper data structure to store plot data
    class PlotData
    {
      /// \brief Unique id of the plot
      public: unsigned int id;

      /// brief Pointer to the plot
      public: IncrementalPlot *plot = nullptr;

      /// \brief A map of container variable ids to their plot curve ids.
      public: std::map<unsigned int, unsigned int> variableCurves;
    };

    /// \internal
    /// \brief PlotCanvas private data
    class PlotCanvasPrivate
    {
      /// \brief Text label
      public: EditableLabel *title;

      /// \brief Splitter that contains all the plots.
      public: QSplitter *plotSplitter;

      /// \brief A map of plot id to plot data;
      public: std::map<unsigned int, PlotData *> plotData;

      /// \brief Pointer to an empty plot.
      public: IncrementalPlot *emptyPlot = nullptr;

      /// \brief Container for all the variableCurves on the Y axis.
      public: VariablePillContainer *yVariableContainer = nullptr;

      /// \brief Delete canvas Qt action
      public: QAction *deleteCanvasAct = nullptr;

      /// \brief Global plot counter.
      public: static unsigned int globalPlotId;
    };
  }
}

// empty plot id
const unsigned int PlotCanvas::EmptyPlot = IGN_UINT32_MAX;

// global plot id count
unsigned int PlotCanvasPrivate::globalPlotId = 0;

/////////////////////////////////////////////////
PlotCanvas::PlotCanvas(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new PlotCanvasPrivate())
{
  this->setObjectName("plotCanvas");

  // Plot title
  this->dataPtr->title = new EditableLabel("Plot Name");

  QHBoxLayout *titleLayout = new QHBoxLayout;
  titleLayout->addWidget(this->dataPtr->title);
  titleLayout->setAlignment(Qt::AlignHCenter);

  // Settings
  QMenu *settingsMenu = new QMenu;
  settingsMenu->setObjectName("material");
  QAction *clearPlotAct = new QAction("Clear all fields", settingsMenu);
  clearPlotAct->setStatusTip(tr("Clear variables and all plots on canvas"));
  connect(clearPlotAct, SIGNAL(triggered()), this, SLOT(OnClearCanvas()));

  this->dataPtr->deleteCanvasAct = new QAction("Delete canvas", settingsMenu);
  this->dataPtr->deleteCanvasAct->setStatusTip(tr("Delete entire canvas"));
  connect(this->dataPtr->deleteCanvasAct, SIGNAL(triggered()), this,
      SLOT(OnDeleteCanvas()));

  QAction *showGridAct = new QAction("Show grid", settingsMenu);
  showGridAct->setStatusTip(tr("Show/hide grid lines on plot"));
  showGridAct->setCheckable(true);

  QAction *showHoverLineAct = new QAction("Show hover line", settingsMenu);
  showHoverLineAct->setStatusTip(tr("Show hover line"));
  showHoverLineAct->setCheckable(true);
  connect(showHoverLineAct, SIGNAL(triggered()), this, SLOT(OnShowHoverLine()));

  settingsMenu->addAction(clearPlotAct);
  settingsMenu->addAction(this->dataPtr->deleteCanvasAct);
  settingsMenu->addAction(showGridAct);
  settingsMenu->addAction(showHoverLineAct);

  QToolButton *settingsButton = new QToolButton();
  settingsButton->setObjectName("plotCanvasTitleTool");
  settingsButton->installEventFilter(this);
  settingsButton->setToolTip(tr("Settings"));
  settingsButton->setIcon(QIcon(":/images/settings.svg"));
  settingsButton->setIconSize(QSize(25, 25));
  settingsButton->setFixedSize(QSize(45, 35));
  settingsButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  settingsButton->setPopupMode(QToolButton::InstantPopup);
  settingsButton->setMenu(settingsMenu);

  QHBoxLayout *settingsLayout = new QHBoxLayout;
  settingsLayout->addWidget(settingsButton);

  QHBoxLayout *titleSettingsLayout = new QHBoxLayout;
  titleSettingsLayout->addLayout(titleLayout);
  titleSettingsLayout->addLayout(settingsLayout);
  titleSettingsLayout->setContentsMargins(0, 0, 0, 0);

  QFrame *titleFrame = new QFrame;
  titleFrame->setObjectName("plotCanvasTitleFrame");
  titleFrame->setLayout(titleSettingsLayout);

  // X and Y variable containers
  VariablePillContainer *xVariableContainer = new VariablePillContainer(this);
  xVariableContainer->SetText("x ");
  xVariableContainer->SetMaxSize(1);
  xVariableContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  xVariableContainer->setContentsMargins(0, 0, 0, 0);
  // \todo: fix hardcoded x axis
  xVariableContainer->AddVariablePill("sim_time");
  xVariableContainer->setEnabled(false);

  this->dataPtr->yVariableContainer = new VariablePillContainer(this);
  this->dataPtr->yVariableContainer->SetText("y ");
  this->dataPtr->yVariableContainer->setSizePolicy(
      QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->dataPtr->yVariableContainer->setContentsMargins(0, 0, 0, 0);

  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableAdded(unsigned int, std::string, unsigned int)),
      this, SLOT(OnAddVariable(unsigned int, std::string, unsigned int)));
  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableRemoved(unsigned int, unsigned int)),
      this, SLOT(OnRemoveVariable(unsigned int, unsigned int)));
  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableMoved(unsigned int, unsigned int)),
      this, SLOT(OnMoveVariable(unsigned int, unsigned int)));
  connect(this->dataPtr->yVariableContainer,
      SIGNAL(VariableLabelChanged(unsigned int, std::string)),
      this, SLOT(OnSetVariableLabel(unsigned int, std::string)));

  QVBoxLayout *variableContainerLayout = new QVBoxLayout;
  variableContainerLayout->addWidget(xVariableContainer);
  variableContainerLayout->addWidget(this->dataPtr->yVariableContainer);
  variableContainerLayout->setSpacing(0);
  variableContainerLayout->setContentsMargins(0, 0, 0, 0);

  // plot
  QScrollArea *plotScrollArea = new QScrollArea(this);
  plotScrollArea->setObjectName("plotScrollArea");
  plotScrollArea->setLineWidth(0);
  plotScrollArea->setFrameShape(QFrame::NoFrame);
  plotScrollArea->setFrameShadow(QFrame::Plain);
  plotScrollArea->setSizePolicy(QSizePolicy::Minimum,
                                QSizePolicy::Expanding);

  plotScrollArea->setWidgetResizable(true);
  plotScrollArea->viewport()->installEventFilter(this);

  QFrame *plotFrame = new QFrame(plotScrollArea);
  plotFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
  plotFrame->setObjectName("plotCanvasPlotFrame");
  QVBoxLayout *plotLayout = new QVBoxLayout;
  plotFrame->setLayout(plotLayout);

  this->dataPtr->plotSplitter = new QSplitter(Qt::Vertical);
  this->dataPtr->plotSplitter->setVisible(false);
  plotLayout->addWidget(this->dataPtr->plotSplitter);

  plotScrollArea->setWidget(plotFrame);

  // empty plot
  this->dataPtr->emptyPlot = new IncrementalPlot(this);
  connect(this->dataPtr->emptyPlot, SIGNAL(VariableAdded(std::string)),
      this, SLOT(OnAddVariable(std::string)));
  plotLayout->addWidget(this->dataPtr->emptyPlot);

  // set initial show grid state
  showGridAct->setChecked(this->dataPtr->emptyPlot->IsShowGrid());
  connect(showGridAct, SIGNAL(triggered()), this, SLOT(OnShowGrid()));

  QFrame *mainFrame = new QFrame;
  mainFrame->setObjectName("plotCanvasFrame");
  QVBoxLayout *mainFrameLayout = new QVBoxLayout;
  mainFrameLayout->addWidget(titleFrame);
  mainFrameLayout->addLayout(variableContainerLayout);
  mainFrameLayout->addWidget(plotScrollArea);
  mainFrameLayout->setContentsMargins(0, 0, 0, 0);
  mainFrame->setLayout(mainFrameLayout);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
PlotCanvas::~PlotCanvas()
{
  this->Clear();
}

/////////////////////////////////////////////////
void PlotCanvas::SetVariableLabel(const unsigned int _id,
    const std::string &_label)
{
  // set new variable labeland let the signals/slots do the work on updating
  // the plot curve
  return this->dataPtr->yVariableContainer->SetVariablePillLabel(_id, _label);
}

/////////////////////////////////////////////////
unsigned int PlotCanvas::AddVariable(const std::string &_variable,
    const unsigned int _plotId)
{
  unsigned int targetId = VariablePill::EmptyVariable;
  if (_plotId != EmptyPlot)
  {
    // find a variable that belongs to the specified plotId and make that the
    // the target variable that the new variable will be added to
    auto it = this->dataPtr->plotData.find(_plotId);
    if (it != this->dataPtr->plotData.end() &&
        !it->second->variableCurves.empty())
    {
      targetId = it->second->variableCurves.begin()->first;
    }
  }

  // add to container and let the signals/slots do the work on adding the
  // a new plot with the curve in the overloaded AddVariable function
  return this->dataPtr->yVariableContainer->AddVariablePill(_variable,
      targetId);
}

/////////////////////////////////////////////////
void PlotCanvas::AddVariable(const unsigned int _id,
    const std::string &_variable, const unsigned int _plotId)
{
  unsigned int plotId;
  if (_plotId == EmptyPlot)
  {
    // create new plot for the variable and add plot to canvas
    plotId = this->AddPlot();
  }
  else
    plotId = _plotId;

  // add variable to existing plot
  auto it = this->dataPtr->plotData.find(plotId);
  if (it == this->dataPtr->plotData.end())
    return;

  PlotData *p = it->second;
  PlotCurveWeakPtr curve = p->plot->AddCurve(_variable);
  auto c = curve.lock();
  if (c)
  {
    p->variableCurves[_id] = c->Id();
  }
  else
  {
    gzerr << "Unable to add curve to plot" << std::endl;
    return;
  }

  // hide initial empty plot
  if (!this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
  {
    this->dataPtr->emptyPlot->setVisible(false);
    this->dataPtr->plotSplitter->setVisible(true);
  }


  if (common::URI::Valid(_variable))
  {
    common::URI uri(_variable);
    std::string schemeStr = uri.Scheme();
    if (schemeStr == "data")
    {
      PlotManager::Instance()->AddIntrospectionCurve(_variable, curve);
       // give it a more compact, friendly name
      // do this after PlotManager AddIntrospectionCurve call!
      std::string label = PlotManager::Instance()->HumanReadableName(_variable);
      this->SetVariableLabel(_id, label);
    }
  }
  else
  {
    PlotManager::Instance()->AddTopicCurve(_variable, curve);
  }
}

/////////////////////////////////////////////////
void PlotCanvas::RemoveVariable(const unsigned int _id,
    const unsigned int _plotId)
{
  auto it = this->dataPtr->plotData.end();
  if (_plotId == EmptyPlot)
  {
    // find which plot the variable belongs to
    for (auto pIt = this->dataPtr->plotData.begin();
        pIt != this->dataPtr->plotData.end(); ++pIt)
    {
      auto v = pIt->second->variableCurves.find(_id);
      if (v != pIt->second->variableCurves.end())
      {
        it = pIt;
        break;
      }
    }
  }
  else
  {
    // get the plot which the variable belongs to
    it = this->dataPtr->plotData.find(_plotId);
  }

  if (it == this->dataPtr->plotData.end())
    return;

  auto v = it->second->variableCurves.find(_id);
  if (v == it->second->variableCurves.end())
    return;

  unsigned int curveId = v->second;

  // remove curve from manager
  PlotCurveWeakPtr plotCurve = it->second->plot->Curve(curveId);
  std::string curveLabel;
  {
    auto pc = plotCurve.lock();
    curveLabel = pc->Label();
  }
  // assume topic name starts with '/'
  if (!curveLabel.empty() && curveLabel[0] == '/')
    PlotManager::Instance()->RemoveTopicCurve(plotCurve);
  else
    PlotManager::Instance()->RemoveIntrospectionCurve(plotCurve);

  // erase from map
  it->second->variableCurves.erase(v);

  // delete whole plot if no more curves
  if (it->second->variableCurves.empty())
  {
    it->second->plot->hide();
    it->second->plot->RemoveCurve(curveId);
    delete it->second->plot;
    delete it->second;
    this->dataPtr->plotData.erase(it);

    this->UpdateAxisLabel();
  }
  else
  {
    it->second->plot->RemoveCurve(curveId);
  }

  if (this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
  {
    this->dataPtr->emptyPlot->setVisible(true);
    this->dataPtr->plotSplitter->setVisible(false);
  }

  // remove from variable pill container
  this->dataPtr->yVariableContainer->RemoveVariablePill(_id);
}


/////////////////////////////////////////////////
unsigned int PlotCanvas::AddPlot()
{
  IncrementalPlot *plot = new IncrementalPlot(this);
  plot->setAutoDelete(false);
  plot->ShowGrid(this->dataPtr->emptyPlot->IsShowGrid());
  plot->ShowHoverLine(this->dataPtr->emptyPlot->IsShowHoverLine());
  connect(plot, SIGNAL(VariableAdded(std::string)), this,
      SLOT(OnAddVariable(std::string)));
  this->dataPtr->plotSplitter->addWidget(plot);

  PlotData *p = new PlotData;
  p->id = this->dataPtr->globalPlotId++;
  p->plot = plot;
  this->dataPtr->plotData[p->id] = p;

  this->UpdateAxisLabel();

  return p->id;
}

/////////////////////////////////////////////////
void PlotCanvas::RemovePlot(const unsigned int _id)
{
  auto it = this->dataPtr->plotData.find(_id);
  if (it == this->dataPtr->plotData.end())
    return;

  // remove the plot if it does not contain any variableCurves (curves)
  if (it->second->variableCurves.empty())
  {
    it->second->plot->hide();
    delete it->second->plot;
    delete it->second;
    this->dataPtr->plotData.erase(it);
    return;
  }

  unsigned int plotId = it->first;
  // remove all variableCurves except last one
  while (it->second->variableCurves.size() > 1)
  {
    auto v = it->second->variableCurves.begin();
    this->RemoveVariable(v->first, plotId);
  }

  // remove last variable - this will also delete the plot which causes
  // plot data iterator to be invalid. So do this last.
  this->RemoveVariable(it->second->variableCurves.begin()->first, plotId);

  this->UpdateAxisLabel();
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
unsigned int PlotCanvas::PlotByVariable(const unsigned int _variableId) const
{
  for (const auto it : this->dataPtr->plotData)
  {
    const auto v = it.second->variableCurves.find(_variableId);
    if (v != it.second->variableCurves.end())
    {
      return it.first;
    }
  }
  return EmptyPlot;
}

/////////////////////////////////////////////////
void PlotCanvas::OnAddVariable(const std::string &_variable)
{
  IncrementalPlot *plot =
      qobject_cast<IncrementalPlot *>(QObject::sender());

  if (!plot)
    return;

  if (plot == this->dataPtr->emptyPlot)
  {
    // add new variable to new plot
    this->AddVariable(_variable);
  }
  else
  {
    for (const auto &it : this->dataPtr->plotData)
    {
      if (plot == it.second->plot)
      {
        // add to existing plot
        this->AddVariable(_variable, it.second->id);
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
void PlotCanvas::OnAddVariable(const unsigned int _id,
    const std::string &_variable, const unsigned int _targetId)
{
  if (_targetId != VariablePill::EmptyVariable)
  {
    // Add a variable to existing plot
    for (const auto it : this->dataPtr->plotData)
    {
      const auto v = it.second->variableCurves.find(_targetId);
      if (v != it.second->variableCurves.end())
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
  this->RemoveVariable(_id);
}

/////////////////////////////////////////////////
void PlotCanvas::OnMoveVariable(const unsigned int _id,
    const unsigned int _targetId)
{
  auto plotIt = this->dataPtr->plotData.end();
  auto targetPlotIt = this->dataPtr->plotData.end();
  unsigned int curveId = 0;

  // find plot which the variable belongs to
  // find target plot (if any) that the variable will be moved to
  for (auto it = this->dataPtr->plotData.begin();
      it != this->dataPtr->plotData.end(); ++it)
  {
    auto v = it->second->variableCurves.find(_id);
    if (v != it->second->variableCurves.end())
    {
      plotIt = it;
      curveId = v->second;
    }

    if (it->second->variableCurves.find(_targetId) !=
        it->second->variableCurves.end())
    {
      targetPlotIt = it;
    }

    if (plotIt != this->dataPtr->plotData.end() &&
        targetPlotIt != this->dataPtr->plotData.end())
      break;
  }

  // detach from old plot and attach to new one
  if (plotIt != this->dataPtr->plotData.end())
  {
    PlotData *p = plotIt->second;

    // detach variable from plot (qwt plot doesn't seem to do anything
    // apart from setting the plot item to null)
    PlotCurvePtr plotCurve = p->plot->DetachCurve(curveId);
    p->variableCurves.erase(p->variableCurves.find(_id));

    if (targetPlotIt != this->dataPtr->plotData.end())
    {
      // attach variable to target plot
      targetPlotIt->second->plot->AttachCurve(plotCurve);
      targetPlotIt->second->variableCurves[_id] = plotCurve->Id();
    }
    else
    {
      // create new plot
      unsigned int plotId = this->AddPlot();
      auto it = this->dataPtr->plotData.find(plotId);
      GZ_ASSERT(it != this->dataPtr->plotData.end(), "Failed to add new plot");
      PlotData *newPlotData = it->second;
      // attach curve to plot
      newPlotData->plot->AttachCurve(plotCurve);
      newPlotData->variableCurves[_id] = plotCurve->Id();

      // hide initial empty plot
      if (!this->dataPtr->plotData.empty() && this->dataPtr->emptyPlot)
      {
        this->dataPtr->emptyPlot->setVisible(false);
        this->dataPtr->plotSplitter->setVisible(true);
      }
    }

    // delete plot if empty
    if (p->variableCurves.empty())
    {
      p->plot->hide();

      // careful about deleting by iterator (plotIt) as it may have been
      // changed if a new plot is added to the vector
      for (auto it = this->dataPtr->plotData.begin();
          it != this->dataPtr->plotData.end(); ++it)
      {
        if (it->second == p)
        {
          this->dataPtr->plotData.erase(it);
          break;
        }
      }

      p->plot->detachItems(QwtPlotItem::Rtti_PlotItem, false);
      delete p->plot;
      delete p;

      this->UpdateAxisLabel();
    }
  }
}

/////////////////////////////////////////////////
void PlotCanvas::OnSetVariableLabel(const unsigned int _id,
    const std::string &_label)
{
  // find plot which the variable belongs to
  for (auto it = this->dataPtr->plotData.begin();
      it != this->dataPtr->plotData.end(); ++it)
  {
    auto v = it->second->variableCurves.find(_id);
    if (v != it->second->variableCurves.end())
    {
      it->second->plot->SetCurveLabel(v->second, _label);
      break;
    }
  }
}

/////////////////////////////////////////////////
void PlotCanvas::Update()
{
  // Update all the plots
  for (auto p : this->dataPtr->plotData)
    p.second->plot->Update();
}

/////////////////////////////////////////////////
void PlotCanvas::Restart()
{
  // tuple of original variable label, variable pointer, plot id to add
  // variable to.
  std::vector<std::tuple<std::string, VariablePill *, unsigned int>>
    variableCurvesToClone;

  // Restart all the plots
  for (const auto &it : this->dataPtr->plotData)
  {
    for (const auto &v : it.second->variableCurves)
    {
      unsigned int variableId = v.first;
      unsigned int curveId = v.second;

      // get variable pill
      VariablePill *variablePill =
          this->dataPtr->yVariableContainer->GetVariablePill(variableId);

      if (!variablePill)
        continue;

      PlotCurveWeakPtr curve = it.second->plot->Curve(curveId);
      auto c = curve.lock();
      if (!c)
        continue;

      if (c->Active())
      {
        c->SetActive(false);
        // remove from manager so they don't get updated any more.
        // assume topic name starts with '/'
        std::string curveLabel = c->Label();
        if (!curveLabel.empty() && curveLabel[0] == '/')
          PlotManager::Instance()->RemoveTopicCurve(c);
        else
          PlotManager::Instance()->RemoveIntrospectionCurve(c);

        // add to the list of variables to clone
        variableCurvesToClone.push_back(std::make_tuple(variablePill->Text(),
            variablePill, it.first));
      }
      // increment curve age.
      c->SetAge(c->Age()+1);

      // update the label of variable pill to show the age of the curve
      std::string varText = variablePill->Text();
      std::stringstream ss;
      if (c->Age() == 1u)
      {
        // update curve label and append curve age to the end
        ss << varText << "_1";
        this->SetVariableLabel(variableId, ss.str());
      }
      else
      {
        // if it's more than one simulation-run old, update the age suffix
        size_t idx = varText.rfind("_");
        if (idx != std::string::npos)
        {
          int age = std::atoi(varText.substr(idx+1).c_str());
          if (age > 0)
          {
            ss << varText.substr(0, idx+1) << c->Age();
            this->SetVariableLabel(variableId, ss.str());
          }
        }
      }
    }
  }

  // add new copy of variable pill with original label
  for (const auto &v : variableCurvesToClone)
  {
    std::string varText;
    VariablePill *varPill;
    unsigned int plotId;
    std::tie(varText, varPill, plotId) = v;

    // add clones of the variable - this will also add it to the plot manager.
    unsigned int id = this->AddVariable(varPill->Name(), plotId);

    // give it the original variable label
    if (varText != varPill->Name())
      this->SetVariableLabel(id, varText);
  }
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
unsigned int PlotCanvas::PlotCount() const
{
  unsigned int plotCount =  this->dataPtr->plotData.size();
  if (this->dataPtr->emptyPlot)
    plotCount += (this->dataPtr->emptyPlot->isVisible() ? 1u : 0u);

  return plotCount;
}

/////////////////////////////////////////////////
unsigned int PlotCanvas::VariableCount(const unsigned int _plotId) const
{
  auto it = this->dataPtr->plotData.find(_plotId);
  if (it == this->dataPtr->plotData.end())
    return 0u;

  return it->second->variableCurves.size();
}

/////////////////////////////////////////////////
PlotCurveWeakPtr PlotCanvas::PlotCurve(const unsigned int _variableId)
{
  for (const auto it : this->dataPtr->plotData)
  {
    const auto v = it.second->variableCurves.find(_variableId);
    if (v != it.second->variableCurves.end())
    {
      return it.second->plot->Curve(v->second);
    }
  }
  return PlotCurveWeakPtr();
}

/////////////////////////////////////////////////
std::vector<IncrementalPlot *> PlotCanvas::Plots() const
{
  std::vector<IncrementalPlot *> plots;
  for (const auto it : this->dataPtr->plotData)
    plots.push_back(it.second->plot);

  return plots;
}

/////////////////////////////////////////////////
void PlotCanvas::OnClearCanvas()
{
  // Ask for confirmation
  std::string msg = "Are you sure you want to clear all fields? \n\n"
        "This will remove all the plots in this canvas. \n";

  QMessageBox msgBox(QMessageBox::Warning, QString("Clear canvas?"),
      QString(msg.c_str()));
  msgBox.setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QPushButton *cancelButton =
      msgBox.addButton("Cancel", QMessageBox::RejectRole);
  QPushButton *clearButton = msgBox.addButton("Clear",
      QMessageBox::AcceptRole);
  msgBox.setDefaultButton(clearButton);
  msgBox.setEscapeButton(cancelButton);
  msgBox.show();
  msgBox.move(this->mapToGlobal(this->pos()));
  msgBox.exec();
  if (msgBox.clickedButton() != clearButton)
    return;

  this->Clear();
}

/////////////////////////////////////////////////
void PlotCanvas::OnDeleteCanvas()
{
  // Ask for confirmation
  std::string msg = "Are you sure you want to delete the entire canvas? \n";

  QMessageBox msgBox(QMessageBox::Warning, QString("Delete canvas?"),
      QString(msg.c_str()));
  msgBox.setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  QPushButton *cancelButton =
      msgBox.addButton("Cancel", QMessageBox::RejectRole);
  QPushButton *deleteButton = msgBox.addButton("Delete",
      QMessageBox::AcceptRole);
  msgBox.setDefaultButton(deleteButton);
  msgBox.setEscapeButton(cancelButton);
  msgBox.show();
  msgBox.move(this->mapToGlobal(this->pos()));
  msgBox.exec();
  if (msgBox.clickedButton() != deleteButton)
    return;

  emit CanvasDeleted();
}

/////////////////////////////////////////////////
void PlotCanvas::OnShowGrid()
{
  this->dataPtr->emptyPlot->ShowGrid(!this->dataPtr->emptyPlot->IsShowGrid());

  for (const auto &it : this->dataPtr->plotData)
    it.second->plot->ShowGrid(!it.second->plot->IsShowGrid());
}

/////////////////////////////////////////////////
void PlotCanvas::OnShowHoverLine()
{
  this->dataPtr->emptyPlot->ShowHoverLine(
      !this->dataPtr->emptyPlot->IsShowHoverLine());

  for (const auto &it : this->dataPtr->plotData)
    it.second->plot->ShowHoverLine(!it.second->plot->IsShowHoverLine());
}

/////////////////////////////////////////////////
void PlotCanvas::UpdateAxisLabel()
{
  // show the x-axis label in the last plot only
  for (int i = 0; i < this->dataPtr->plotSplitter->count(); ++i)
  {
    IncrementalPlot *p =
        qobject_cast<IncrementalPlot *>(this->dataPtr->plotSplitter->widget(i));

    if (p)
    {
      p->ShowAxisLabel(IncrementalPlot::X_BOTTOM_AXIS,
          i == (this->dataPtr->plotSplitter->count()-1));
    }
  }
}

/////////////////////////////////////////////////
void PlotCanvas::SetDeleteCanvasEnabled(const bool _enable)
{
  if (this->dataPtr->deleteCanvasAct)
    this->dataPtr->deleteCanvasAct->setEnabled(_enable);
}

/////////////////////////////////////////////////
std::string PlotCanvas::Title() const
{
  return this->dataPtr->title->Text();
}

/////////////////////////////////////////////////
void PlotCanvas::Export(const std::string &_dirName,
    const FileType _type) const
{
  std::string title = this->Title();

  // Cleanup the title
  std::replace(title.begin(), title.end(), '/', '_');
  std::replace(title.begin(), title.end(), '?', ':');

  std::string filePrefix = _dirName + "/" + title;

  if (_type == FileType::PDFFile)
    this->ExportPDF(filePrefix);
  else if (_type == FileType::CSVFile)
    this->ExportCSV(filePrefix);
}

/////////////////////////////////////////////////
void PlotCanvas::ExportPDF(const std::string &_filePrefix) const
{
  // Render the plot to a PDF
  int index = 0;
  for (const auto it : this->dataPtr->plotData)
  {
    std::string suffix =
        this->dataPtr->plotData.size() > 1 ? std::to_string(index) : "";

    std::string filename =
      common::unique_file_path(_filePrefix + suffix, "pdf");

    IncrementalPlot *plot = it.second->plot;

    QSizeF docSize(plot->canvas()->width() + plot->legend()->width(),
                   plot->canvas()->height());

    QwtPlotRenderer renderer;
    renderer.renderDocument(plot, QString(filename.c_str()), docSize, 20);

    gzmsg << "Plot exported to file [" << filename << "]" << std::endl;

    index++;
  }
}

/////////////////////////////////////////////////
void PlotCanvas::ExportCSV(const std::string &_filePrefix) const
{
  // Save data from each curve into a separate file.
  for (const auto it : this->dataPtr->plotData)
  {
    for (const auto &curve : it.second->plot->Curves())
    {
      auto c = curve.lock();
      if (!c)
        continue;

      // Cleanup the label
      std::string label = c->Label();
      std::replace(label.begin(), label.end(), '/', '_');
      std::replace(label.begin(), label.end(), '?', ':');

      std::string filename =
          common::unique_file_path(_filePrefix + "-" + label, "csv");

      std::ofstream out(filename);
      // \todo: fix hardcoded sim_time
      out << "sim_time, " << c->Label() << std::endl;
      for (unsigned int j = 0; j < c->Size(); ++j)
      {
        ignition::math::Vector2d pt = c->Point(j);
        out << pt.X() << ", " << pt.Y() << std::endl;
      }
      out.close();

      gzmsg << "Plot exported to file [" << filename << "]" << std::endl;
    }
  }
}
