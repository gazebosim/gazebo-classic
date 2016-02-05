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
#ifndef _GAZEBO_GUI_PLOTCANVAS_HH_
#define _GAZEBO_GUI_PLOTCANVAS_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/PlotManager.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotCanvasPrivate;

    /// \brief Plot canvas
    class GZ_GUI_VISIBLE PlotCanvas : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent widget.
      public: PlotCanvas(QWidget *_parent);

      /// \brief Destructor.
      public: virtual ~PlotCanvas();

      /// \brief Add a new variable to a new plot.
      /// \param[in] _variable Name of the variable.
      /// \return Unique id of the variable
      public: unsigned int AddVariable(const std::string &_variable);

      /// \brief Remove a variable from a plot. This will search through all
      /// plots for the variable and remove it from the plot if found.
      /// \param[in] _id Unique id of the variable
      public: void RemoveVariable(const unsigned int _id);

      /// \brief Remove a variable from a plot.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _plotId Unique if of plot to remove the variable from.
      public: void RemoveVariable(const unsigned int _id,
          const unsigned int _plotId);

      /// \brief Add a new plot to the canvas.
      /// \return Unique id of the plot
      public: unsigned int AddPlot();

      /// \brief Remove a plot from the canvas.
      /// \param[in] _id Unique id of the plot
      public: void RemovePlot(const unsigned int _plotId);

      /// \brief Get the number of plots in this canvas.
      /// \return Number of plots
      public: unsigned int PlotCount() const;

      /// \brief Get the number of variables in a plot.
      /// \param[in] _plotId Unique plot id
      /// \return Number of variables
      public: unsigned int VariableCount(const unsigned int _plotId) const;

      /// \brief Update plots.
      public: void Update();

      /// \brief Get the plot id which the variable is plotted in
      /// \param[in] _id Unique id of the variable
      /// \return _id Unique id of the plot
      // public: unsigned int Plot(const unsigned int _variableId) const;

      /// \brief Get all the plots in this canvas.
      /// \return A list of plots in this canvas.
      public: std::vector<IncrementalPlot *> Plots();

      /// \brief Get the curve associated with the variable
      /// \param[in] _id Unique id of the variable
      /// \return A pointer to the PlotCurve object.
      public: PlotCurveWeakPtr PlotCurve(const unsigned int _variableId);

      /// \brief Clear the canvas and remove all variables and plots.
      public: void Clear();

      /// \brief Restart all the plots in this canvas
      public: void RestartPlots();

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      public: virtual bool eventFilter(QObject *_o, QEvent *_e);

      /// \brief Add a variable to an existing plot. Note this function
      /// only updates the plot but not the variable pill container.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _variable Name of the variable
      /// \param[in] _plot Unique id of the plot to add the variable to.
      private: void AddVariable(const unsigned int _id,
          const std::string &_variable, const unsigned int _plotId);

      /// \brief Add a new variable to a new plot. Note this function
      /// only updates the plot but not the variable pill container.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _variable Name of the variable
      private: void AddVariable(const unsigned int _id,
          const std::string &_variable);

      /// \brief Qt signal to request self-deletion.
      Q_SIGNALS: void CanvasDeleted();

      /// \brief Qt Callback when a new variable has been added.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _variable Name of the variable
      /// \param[in] _targetId Unique id of the target variable that the
      /// the variable is now co-located with.
      private slots: void OnAddVariable(const unsigned int _id,
          const std::string &_variable, const unsigned int _targetId);

      /// \brief Qt Callback when a variable has been removed.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _targetId Unique id of the target variable that the
      /// the variable was co-located with.
      private slots: void OnRemoveVariable(const unsigned int _id,
                const unsigned int _targetId);

      /// \brief Qt Callback when a variable has moved from one plot to another.
      /// \param[in] _id Unique id of the variable that has moved.
      /// \param[in] _targetId Unique id of the target variable that the
      /// the moved variable is now co-located with.
      private slots: void OnMoveVariable(const unsigned int _id,
          const unsigned int _targetId);

      /// \brief Qt Callback to clear all variable and plots on canvas.
      private slots: void OnClearCanvas();

      /// \brief Qt Callback to delete entire canvas.
      private slots: void OnDeleteCanvas();

      /// \brief Empty plot used to indicate non-existent plot.
      public: static unsigned int EMPTY_PLOT;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotCanvasPrivate> dataPtr;

      // TODO remove me
      private: void debug();
    };
  }
}
#endif
