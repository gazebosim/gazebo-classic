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
#ifndef GAZEBO_PLUGINS_KEYBOARDGUIPLUGIN_HH_
#define GAZEBO_PLUGINS_KEYBOARDGUIPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>

// See: https://bugreports.qt-project.org/browse/QTBUG-22829
#ifndef Q_MOC_RUN
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  // Forward declare private data class
  class KeyboardGUIPluginPrivate;

  /// \brief A GUI plugin that captures key strokes from gzclient GUI
  /// and publishes over gz transport topic `~/keyboard/keypress`
  class GAZEBO_VISIBLE KeyboardGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: KeyboardGUIPlugin();

    /// \brief Destructor.
    public: virtual ~KeyboardGUIPlugin();

    /// \brief Callback for a key press event.
    /// \param[in] _event Key event
    protected: void OnKeyPress(const gazebo::common::KeyEvent &_event);

    /// \brief Qt event filter used to filter child widget events.
    /// \param[in] _obj Object that is watched by the event filter.
    /// \param[in] _event Qt event.
    /// \return True if the event is handled.
    private: bool eventFilter(QObject *_obj, QEvent *_event);

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<KeyboardGUIPluginPrivate> dataPtr;
  };
}

#endif
