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
#include "gazebo/msgs/msgs.hh"
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

      public: PlotPalette(QWidget *_parent);
      public: ~PlotPalette();

      signals: void InsertModelSignal(const std::string &_name);
      signals: void RemoveModelSignal(const std::string &_name);

      private: void FillTopicsTop();
      private: void FillSimBottom();
      private: void FillModel(const std::string &_model);
      private: void FillTopicFromMsg(google::protobuf::Message *_msg,
          const std::string &_scope, const unsigned int _level,
          QVBoxLayout *_parentLayout);

      private slots: void OnModelClicked(const std::string &_model);
      private slots: void OnTopicClicked(const std::string &_topic);
      private slots: void OnInsertModelSignal(const std::string &_name);
      private slots: void OnRemoveModelSignal(const std::string &_name);

      private: void OnModel(ConstModelPtr &_msg);
      private: void OnResponse(ConstResponsePtr &_msg);
      private: void OnRequest(ConstRequestPtr &_msg);

      public: static const std::vector<std::string> ModelProperties;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotPalettePrivate> dataPtr;
    };

    // Forward declare private data class
    class ItemConfigWidgetPrivate;

    class GZ_GUI_VISIBLE ItemConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      public: ItemConfigWidget(const std::string &_text,
          const unsigned int _level = 0);
      public: ~ItemConfigWidget();
      public: std::string Text() const;
      public: void SetDraggable(const bool _draggable);
      public: void SetPlotInfo(const std::string &_info);
      signals: void Clicked(const std::string &);
      protected: virtual void mouseReleaseEvent(QMouseEvent *_event);
      protected: virtual void mousePressEvent(QMouseEvent *_event);
      protected: virtual void enterEvent(QEvent *_event);
      protected: virtual void leaveEvent(QEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<ItemConfigWidgetPrivate> dataPtr;
    };
  }
}
#endif
