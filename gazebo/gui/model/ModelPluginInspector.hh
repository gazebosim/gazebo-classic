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

#ifndef _GAZEBO_MODEL_PLUGIN_INSPECTOR_HH_
#define _GAZEBO_MODEL_PLUGIN_INSPECTOR_HH_

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class ModelPluginInspectorPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \brief Inspector for model plugin properties.
    class GZ_GUI_MODEL_VISIBLE ModelPluginInspector : public QDialog
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent QWidget.
      public: ModelPluginInspector(QWidget *_parent = 0);

      /// \brief Destructor
      public: ~ModelPluginInspector();

      /// \brief Update the config widget with a msg.
      /// \param[in] _pluginMsg Plugin message.
      public: void Update(ConstPluginPtr _pluginMsg);

      /// \brief Qt event emiited when the mouse enters this widget.
      /// \param[in] _event Qt event.
      protected: virtual void enterEvent(QEvent *_event);

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \internal
      /// \brief Pointer to private data.
      private: ModelPluginInspectorPrivate *dataPtr;
    };
    /// \}
  }
}

#endif
