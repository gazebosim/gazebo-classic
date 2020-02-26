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

#include <ignition/transport/Node.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
  /// \brief A GUI plugin that demos the ignition::math::Matrix4<T>::LookAt
  /// function. The plugin offers 3 input fields (XYZ) for each of 3 vectors:
  /// the position of the eye, the position of the target and the desired up
  /// direction. See more documentation on ignition math.
  ///
  /// It works with the lookat_demo.world, and expects that world to have 3
  /// models:
  ///
  /// 1. "frame": A model representing the resulting frame, the one which
  /// "looks at"
  ///
  /// 2. "target": A yellow sphere representing the target which is being
  /// "looked at"
  ///
  /// 3. "desired_z": A single axis, representing the desired Z axis.
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

    // Place ignition::transport objects at the end of this file to
    // guarantee they are destructed first.

    /// \brief Node for communication.
    private: ignition::transport::Node nodeIgn;

    /// \brief To publish model modify messages.
    private: ignition::transport::Node::Publisher modelModifyPubIgn;
  };
}

#endif
