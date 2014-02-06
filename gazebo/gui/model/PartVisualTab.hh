/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _PART_VISUAL_TAB_HH_
#define _PART_VISUAL_TAB_HH_

#include <string>

#include "gazebo/math/Pose.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{

  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class PartVisualTab PartVisualTab.hh
    /// \brief A tab for configuring visual properties of a part.
    class PartVisualTab : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartVisualTab();

      /// \brief Destructor
      public: ~PartVisualTab();

      /// \brief Widget that display visuals' properties.
      private: QTreeWidget *visualsTreeWidget;

      /// \brief Counter for the number of visuals.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of visual items to their id.
      private: std::map<int, QTreeWidgetItem *> visualItems;

      /// \brief Qt callback when a visual is to be added.
      private slots: void OnAddVisual();

      /// \brief Qt callback when a visual is to be removed.
      /// \param[in] _item Item to be removed.
      private slots: void OnRemoveVisual(int);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);
    };
  }
}
#endif
