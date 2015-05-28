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
#ifndef _GAZEBO_GUI_CESSNA_PLUGIN_HH_
#define _GAZEBO_GUI_CESSNA_PLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  /// \brief A GUI plugin that controls the Cessna model using the keyboard.
  /// If you are reading this, feel free to improve this plugin by adding
  /// graphical widgets to make the demo more interesting and fun.
  class GAZEBO_VISIBLE CessnaGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: CessnaGUIPlugin();

    /// \brief Destructor.
    public: virtual ~CessnaGUIPlugin();

    // Documentation inherited.
    public: void Load(sdf::ElementPtr _elem);

    /// \brief Increase the propeller RPMs.
    public slots: void OnIncreaseThrust();

    /// \brief Decrease the propeller RPMs.
    public slots: void OnDecreaseThrust();

    /// \brief Increase the flaps angle.
    public slots: void OnIncreaseFlaps();

    /// \brief Decrease the flaps angle.
    public slots: void OnDecreaseFlaps();

    /// \brief Increase the elevators angle
    public slots: void OnIncreaseElevators();

    /// \brief Decrease the elevators angle.
    public slots: void OnDecreaseElevators();

    /// \brief Increase the rudder angle.
    public slots: void OnIncreaseRudder();

    /// \brief Decrease the rudder angle.
    public slots: void OnDecreaseRudder();

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Control publisher.
    private: transport::PublisherPtr controlPub;

    /// \brief Target thrust percentage.
    private: int targetThrust = 0;

    /// \brief Target flaps angle in degrees.
    private: math::Angle targetFlaps = math::Angle::Zero;

    /// \brief Target elevators angle in degrees.
    private: math::Angle targetElevators = math::Angle::Zero;

    /// \brief Target rudder angle in degrees.
    private: math::Angle targetRudder = math::Angle::Zero;

    /// \brief Angle increment/decrement each time a key is pressed;
    private: math::Angle angleStep;
  };
}

#endif
