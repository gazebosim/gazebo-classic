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
#ifndef _GAZEBO_GUI_PLOT_PLOTPALETTE_HH_
#define _GAZEBO_GUI_PLOT_PLOTPALETTE_HH_

#include <memory>

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/qt.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private data class
    class PlotPalettePrivate;

    /// \brief A special list widget that allows dragging of items from it to a
    /// plot
    class GZ_GUI_VISIBLE PlotPalette : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to parent widget.
      public: PlotPalette(QWidget *_parent);

      /// \brief Destructor
      public: ~PlotPalette();

      /// \brief Fill the top panel of the topics tab.
      private: void FillTopicsTop();

      /// \brief Fill the bottom panel of the sim tab.
      private: void FillSimBottom();

      /// \brief Fill the bottom panel with plottable properties from a topic.
      /// The panel to be filled will depend on the widget given, and could be
      /// for example the bottom pane of the topics tab or the bottom pane of
      /// search tab.
      /// \param[in] _topic Scoped topic.
      /// \param[out] _widget Config widget to be filled.
      private: void FillTopicsBottom(const std::string &_topic,
          ConfigWidget *_widget);

      /// \brief Fill a layout with properties from a protobuf message.
      /// Only plottable fields such as int, double and bool are displayed.
      /// \param[in] _msg A basic message from the topic's message type.
      /// \param[in] _scope The topic name scope.
      /// \param[in] _level Level of message within the tree.
      /// \param[out _parentLayout Layout to be filled.
      private: void FillFromMsg(google::protobuf::Message *_msg,
          const std::string &_scope, const unsigned int _level,
          QVBoxLayout *_parentLayout);

      /// \brief Callback when a topic has been clicked from the topics tab.
      /// \param[in] _topic Topic scoped name.
      private slots: void OnTopicClicked(const std::string &_topic);

      /// \brief Callback when a topic has been clicked from the search tab.
      /// \param[in] _topic Topic scoped name.
      private slots: void OnTopicSearchClicked(const std::string &_topic);

      /// \brief Callback when the user has modified the search.
      /// \param[in] _search Latest search.
      private slots: void UpdateSearch(const QString &_search);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotPalettePrivate> dataPtr;
    };

    // Forward declare private data class
    class PlotChildConfigWidgetPrivate;

    /// \brief Config widget with properties helpful for the plot palette.
    class GZ_GUI_VISIBLE PlotChildConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _text Text to go on the label.
      /// \param[in] _level Level in the hierarchy.
      public: PlotChildConfigWidget(const std::string &_text,
          const unsigned int _level = 0);

      /// \brief Destructor.
      public: ~PlotChildConfigWidget();

      /// \brief Get the widget's text.
      /// \return The text from the widget's label.
      public: std::string Text() const;

      /// \brief Set whether this can be dragged onto the plot. Widgets
      /// are not draggable by default.
      /// \param[in] _draggable True to make it draggable.
      public: void SetDraggable(const bool _draggable);

      /// \brief Set the information which this item carries when dragged.
      /// \param[in] _info The information.
      public: void SetPlotInfo(const std::string &_info);

      /// \brief Signal that this has been clicked.
      signals: void Clicked(const std::string &);

      // Documentation inherited.
      protected: virtual void mouseReleaseEvent(QMouseEvent *_event);

      // Documentation inherited.
      protected: virtual void mousePressEvent(QMouseEvent *_event);

      // Documentation inherited.
      protected: virtual void enterEvent(QEvent *_event);

      // Documentation inherited.
      protected: virtual void leaveEvent(QEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotChildConfigWidgetPrivate> dataPtr;
    };
  }
}
#endif

