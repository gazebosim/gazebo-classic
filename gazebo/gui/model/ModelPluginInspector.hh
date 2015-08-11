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

#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/ConfigWidget.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Inspector for model plugin properties.
    class ModelPluginInspector : public QDialog
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

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt signal emitted to indicate that changes should be applied
      /// and the inspector closed.
      Q_SIGNALS: void Accepted();

      /// \brief Qt callback when the Cancel button is pressed.
      private slots: void OnCancel();

      /// \brief Qt callback when the Apply button is pressed.
      private slots: void OnApply();

      /// \brief Qt callback when the Ok button is pressed.
      private slots: void OnOK();

      /// \brief Config widget for configuring properties.
      private: ConfigWidget *configWidget;
    };
    /// \}
  }
}

#endif
