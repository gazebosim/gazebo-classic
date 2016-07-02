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
#ifndef GAZEBO_GUI_PLOT_PLOTCANVAS_HH_
#define GAZEBO_GUI_PLOT_PLOTCANVAS_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Vector2.hh>

#include "gazebo/gui/qt.h"
#include "gazebo/gui/plot/ExportDialog.hh"
#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class IncrementalPlot;
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

      /// \brief Set the label of a variable.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _label New variable label.
      public: void SetVariableLabel(const unsigned int _id,
          const std::string &_label);

      /// \brief Add a new variable to a plot.
      /// \param[in] _variable Name of the variable.
      /// \param[in] _plotId Unique id of the plot to add the variable to.
      /// \return Unique id of the variable
      public: unsigned int AddVariable(const std::string &_variable,
          const unsigned int _plotId = EmptyPlot);

      /// \brief Remove a variable from a plot.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _plotId Unique id of plot to remove the variable from.
      /// If EmptyPlot is specified, the function will search through all
      /// plots for the variable and remove it from the plot if found.
      public: void RemoveVariable(const unsigned int _id,
          const unsigned int _plotId = EmptyPlot);

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

      /// \brief Restart plotting. A new plot curve will be created for each
      /// variable in the plot. Existing plot curves will no longer be updated.
      public: void Restart();

      /// \brief Update plots and curves with new data.
      public: void Update();

      /// \brief Get the plot id which the variable is plotted in
      /// \param[in] _id Unique id of the variable
      /// \return Unique id of the plot
      public: unsigned int PlotByVariable(const unsigned int _variableId) const;

      /// \brief Get all the plots in this canvas.
      /// \return A list of plots in this canvas.
      public: std::vector<IncrementalPlot *> Plots() const;

      /// \brief Get the curve associated with the variable
      /// \param[in] _id Unique id of the variable
      /// \return A pointer to the PlotCurve object.
      public: PlotCurveWeakPtr PlotCurve(const unsigned int _variableId);

      /// \brief Clear the canvas and remove all variables and plots.
      public: void Clear();

      /// \brief Set whether or not to enable the delete canvas option in
      /// settings
      /// \param[in] _enable True to enable delete canvas option
      public: void SetDeleteCanvasEnabled(const bool _enable);

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      /// \return True if event was handled.
      public: virtual bool eventFilter(QObject *_o, QEvent *_event);

      /// \brief Get the title of the plot
      /// \return Title of the plot
      public: std::string Title() const;

      /// \brief Export to PDF files in the given directory.
      /// \param[in] _dirName Name of the directory to save the files into.
      /// \param[in] _type File type, such as PDFFile or CSVFile.
      public: void Export(const std::string &_dirName,
          const FileType _type) const;

      /// \brief Export to PDF files in the given directory.
      /// \param[in] _dirName Name of the directory to save the PDF files
      /// into.
      private: void ExportPDF(const std::string &_dirName) const;

      /// \brief Export to CSV files in the given directory.
      /// \param[in] _dirName Name of the directory to save the CSV files
      /// into.
      private: void ExportCSV(const std::string &_dirName) const;

      /// \brief Add a variable to a plot. Note this function
      /// only updates the plot but not the variable pill container.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _variable Name of the variable
      /// \param[in] _plotId Unique id of the plot to add the variable to.
      /// EmptyPlot means add to a new plot.
      private: void AddVariable(const unsigned int _id,
          const std::string &_variable,
          const unsigned int _plotId = EmptyPlot);

      /// \brief Update the axis label for plots in the canvas.
      /// Currently used to determine which plot will display the x-axis label
      /// when plots are added/removed
      private: void UpdateAxisLabel();

      /// \brief Qt signal to request self-deletion.
      Q_SIGNALS: void CanvasDeleted();

      /// \brief Qt Callback when a new variable has been dropped into a plot.
      /// \param[in] _variable Name of the variable
      private slots: void OnAddVariable(const std::string &_variable);

      /// \brief Qt Callback when a new variable has been added.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _variable Name of the variable
      /// \param[in] _targetId Unique id of the target variable that the
      /// variable is now co-located with.
      private slots: void OnAddVariable(const unsigned int _id,
          const std::string &_variable, const unsigned int _targetId);

      /// \brief Qt Callback when a variable has been removed.
      /// \param[in] _id Unique id of the variable
      /// \param[in] _targetId Unique id of the target variable that the
      /// variable was co-located with.
      private slots: void OnRemoveVariable(const unsigned int _id,
                const unsigned int _targetId);

      /// \brief Qt Callback when a variable has moved from one plot to another.
      /// \param[in] _id Unique id of the variable that has moved.
      /// \param[in] _targetId Unique id of the target variable that the
      /// moved variable is now co-located with.
      private slots: void OnMoveVariable(const unsigned int _id,
          const unsigned int _targetId);

      /// \brief Qt Callback when a variable label has changed.
      /// \param[in] _id Unique id of the variable whose label has changed.
      /// \param[in] _label New label text.
      private slots: void OnSetVariableLabel(const unsigned int _id,
          const std::string &_label);

      /// \brief Qt Callback to clear all variable and plots on canvas.
      private slots: void OnClearCanvas();

      /// \brief Qt Callback to delete entire canvas.
      private slots: void OnDeleteCanvas();

      /// \brief Qt Callback to show/hide grids on plot.
      private slots: void OnShowGrid();

      /// \brief Qt Callback to show/hide hover line on plot.
      private slots: void OnShowHoverLine();

      /// \brief Empty plot used to indicate non-existent plot.
      public: static const unsigned int EmptyPlot;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotCanvasPrivate> dataPtr;
    };
  }
}
#endif
