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

#include <mutex>
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
  ///
  /// Keyboard controls:
  /// w         Increase thrust (+10 %)
  /// s         Decrease thrust (-10 %)
  /// d         Increase rudder angle (+1 degree)
  /// a         Decrease rudder angle (-1 degree)
  /// Left-Key  Left roll (+1 degree)
  /// Right-Key Right roll (+1 degree)
  /// Up-Key    Pitch down (+1 degree)
  /// Down-Key  Pitch up (+1 degree)
  /// 1         Preset for take-off
  /// 2         Preset for cruise
  /// 3         Preset for landing
  class GAZEBO_VISIBLE CessnaGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: CessnaGUIPlugin();

    /// \brief Destructor.
    public: virtual ~CessnaGUIPlugin();

    /// \brief Callback that receives a control message from
    /// the ~/cessna_c172/state topic.
    /// \param[in] _msg State msg.
    private: void OnState(ConstCessnaPtr &_msg);

    /// \brief Increase the propeller RPMs.
    private slots: void OnIncreaseThrust();

    /// \brief Decrease the propeller RPMs.
    private slots: void OnDecreaseThrust();

    /// \brief Increase the flaps angle.
    private slots: void OnIncreaseFlaps();

    /// \brief Decrease the flaps angle.
    private slots: void OnDecreaseFlaps();

    /// \brief Increase Roll.
    private slots: void OnIncreaseRoll();

    /// \brief Decrease Roll.
    private slots: void OnDecreaseRoll();

    /// \brief Increase the elevators angle.
    private slots: void OnIncreaseElevators();

    /// \brief Decrease the elevators angle.
    private slots: void OnDecreaseElevators();

    /// \brief Increase the rudder angle.
    private slots: void OnIncreaseRudder();

    /// \brief Decrease the rudder angle.
    private slots: void OnDecreaseRudder();

    /// \brief Take-off preset.
    private slots: void OnPresetTakeOff();

    /// \brief Cruise preset.
    private slots: void OnPresetCruise();

    /// \brief Landing preset.
    private slots: void OnPresetLanding();

    /// \brief SDF for this plugin.
    private: sdf::ElementPtr sdf;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Control publisher.
    private: transport::PublisherPtr controlPub;

    /// \brief State subscriber.
    private: transport::SubscriberPtr stateSub;

    /// \brief Angle increment/decrement each time a key is pressed;
    private: math::Angle angleStep;

    /// \brief State received from the Cessna plugin.
    private: msgs::Cessna state;

    /// \brief Protection.
    private: std::mutex mutex;
  };
}

#endif
