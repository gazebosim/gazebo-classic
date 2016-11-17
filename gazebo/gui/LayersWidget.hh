/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_LAYERSWIDGET_HH_
#define GAZEBO_GUI_LAYERSWIDGET_HH_

#include "gazebo/common/Events.hh"
#include "gazebo/util/system.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    // Forward declare private class
    class LayersWidgetPrivate;

    /// \brief A widget that manages visualization layers. This widget is
    /// added to the left-hand tabset.
    class GZ_GUI_VISIBLE LayersWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget pointer.
      public: LayersWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~LayersWidget();

      /// \brief QT callback, triggered when a layer is selected.
      /// \param[in] _layer Pointer to the widget that was selected.
      private slots: void OnLayerSelected(QListWidgetItem *_layer);

      /// \brief Gazebo callback, triggered when a new layer is added.
      /// \param[in] _layer Number of the new layer
      private: void OnNewLayer(const int32_t _layer);

      /// \internal
      /// \brief Pointer to private data.
      private: LayersWidgetPrivate *dataPtr;
    };
  }
}
#endif
