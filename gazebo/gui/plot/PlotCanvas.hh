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

#include "gazebo/gui/qt.h"
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

      /// \brief Used to filter scroll wheel events.
      /// \param[in] _o Object that receives the event.
      /// \param[in] _event Pointer to the event.
      public: virtual bool eventFilter(QObject *_o, QEvent *_e);

      /// \brief Update plots.
      private slots: void Update();

      /// \brief Qt Callback when a new variable has been added.
      /// \param[_id] _id Unique id of the variable
      /// \param[_id] _targetId Unique id of the target variable that the
      /// the variable is now co-located with.
      private slots: void OnAddVariable(const unsigned int _id,
          const unsigned int _targetId, const std::string &_variable);

      /// \brief Qt Callback when a variable has been removed.
      /// \param[_id] _id Unique id of the variable
      /// \param[_id] _targetId Unique id of the target variable that the
      /// the variable was co-located with.
      private slots: void OnRemoveVariable(const unsigned int _id,
                const unsigned int _targetId);

      /// \brief Qt Callback when a variable has moved from one plot to another.
      /// \param[_id] _id Unique id of the variable that has moved.
      /// \param[_id] _targetId Unique id of the target variable that the
      /// the moved variable is now co-located with.
      private slots: void OnMoveVariable(const unsigned int _id,
          const unsigned int _targetId);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<PlotCanvasPrivate> dataPtr;

      // TODO remove me
      private: void debug();
    };
  }
}
#endif
