/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_LAYERS_WIDGET_HH_
#define _GAZEBO_LAYERS_WIDGET_HH_

#include <vector>

#include "gazebo/common/Events.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GZ_GUI_VISIBLE LayersWidget : public QWidget
    {
      Q_OBJECT
      public: LayersWidget(QWidget *_parent = 0);
      public: virtual ~LayersWidget();

      public slots: void OnLayerSelected(QListWidgetItem *_layer);
      private: void OnNewLayer(const int32_t _layer);

      private: QListWidget *layerList;

      /// \brief Event connections
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
