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
#ifndef _GAZEBO_TRANSPORTER_PLUGIN_HH_
#define _GAZEBO_TRANSPORTER_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  // Forward declare private data class
  class TransporterPluginPrivate;

  /// \brief A plugin that allows models to transport (teleport) to
  /// a new location. A transporter plugin uses multiple named <pads>, where
  /// each <pad> defines an outgoing region, an incoming pose, and
  /// a destination pad.
  ///
  /// When a model enters a pad's outgoing region it is moved to the
  /// destination pad's incoming pose. This means the transporter plugin is
  /// only useful if at least two pads are defined.
  ///
  /// The following is example usage in SDF:
  ///
  /** \verbatim
    <plugin filename="libTransporterPlugin.so" name="transporter">
      <!-- Topic that facilitates manual activation of a pad. An
      activation message is only meaningful for pad's with manual
      activation. An activation message consists of
      a string message on this topic with a pad's name to activate.
      See examples/stand_alone/transporter.cc for an example for
      triggering a manual pad. -->
      <activation_topic>~/transporter</activation_topic>

      <!-- Pad 1, which is automatically updated. This means any model
      that enter the outgoing region will be moved. -->
      <pad name="pad1">
        <destination>pad2</destination>
        <activation>auto</activation>

        <outgoing>
          <min>-.5 -.5 0</min>
          <max>.5 .5 1</max>
        </outgoing>

        <incoming>
          <pose>2 3.5 0 0 0 0</pose>
        </incoming>
      </pad>

      <!-- Pad 2 is manually updated. This means a model will be
      moved only if it is in the outgoing region and an activation
      message has been received. An activation message consists of
      a string message on the <activation_topic> topic with
      the pad's name. -->
      <pad name="pad2">
        <destination>pad1</destination>
        <activation>manual</activation>

        <outgoing>
          <min>-.5 3.0 0</min>
          <max>.5 4.0 1</max>
        </outgoing>

        <incoming>
          <pose>2 0 0 0 0 0</pose>
        </incoming>
      </pad>
    </plugin>
   \endverbatim */
  class GAZEBO_VISIBLE TransporterPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: TransporterPlugin();

    /// \brief Destructor.
    public: virtual ~TransporterPlugin();

    // Documentation inherited
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Update the plugin. This is updated every iteration of
    /// simulation.
    private: void Update();

    /// \brief Callback that receives activation messages.
    /// \param[in] _msg String message that indicates what transporter pad
    /// was activated.
    private: void OnActivation(ConstGzStringPtr &_msg);

    /// \brief Private data pointer.
    private: TransporterPluginPrivate *dataPtr;
  };
}
#endif
