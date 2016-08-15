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
#ifndef GAZEBO_PLUGINS_LOOKATDEMOPLUGIN_HH_
#define GAZEBO_PLUGINS_LOOKATDEMOPLUGIN_HH_

#include <gazebo/transport/transport.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  /// \brief A GUI plugin that displays a timer. Control of this timer
  /// is managed via a topic that is ~/timer_control by default. The
  /// topic may be specified in the plugin's SDF. The topic receives
  /// string messages where:
  ///
  ///     "start" == start the timer
  ///     "stop" == stop the timer
  ///     "reset" == reset the timer
  ///
  /// <plugin name="timer_plugin" filename="libLookAtDemoPlugin.so">
  ///   <topic>~/my_timer_control</topic>
  ///   <pos>pixel_x_pos pixel_y_pos</pos>
  ///   <size>pixel_width pixel_height</size>
  ///   <start_stop_button>visible</start_stop_button>
  ///   <reset_button>visible</reset_button>
  /// </plugin>
  ///
  /// Note that the size has to be large enough to fit all widgets.
  class GAZEBO_VISIBLE LookAtDemoPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: LookAtDemoPlugin();

    /// \brief Destructor
    public: virtual ~LookAtDemoPlugin();

    // Documentation inherited
    public: void Load(sdf::ElementPtr /*_elem*/);

    /// \brief Callback when a value changes.
    /// \param[in] _newValue The new value.
    private slots: void OnChange(const double _newValue);

    /// \brief Box holding the eye X value.
    private: QDoubleSpinBox *eyeX;

    /// \brief Box holding the eye Y value.
    private: QDoubleSpinBox *eyeY;

    /// \brief Box holding the eye Z value.
    private: QDoubleSpinBox *eyeZ;

    /// \brief Box holding the target X value.
    private: QDoubleSpinBox *targetX;

    /// \brief Box holding the target Y value.
    private: QDoubleSpinBox *targetY;

    /// \brief Box holding the target Z value.
    private: QDoubleSpinBox *targetZ;

    /// \brief Box holding the up X value.
    private: QDoubleSpinBox *upX;

    /// \brief Box holding the up Y value.
    private: QDoubleSpinBox *upY;

    /// \brief Box holding the up Z value.
    private: QDoubleSpinBox *upZ;

    /// \brief Node for communication.
    private: transport::NodePtr node;

    /// \brief To publish model modify messages.
    private: transport::PublisherPtr modelModifyPub;
  };
}

#endif
