/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_TOUCHPLUGIN_HH_
#define GAZEBO_PLUGINS_TOUCHPLUGIN_HH_

#include <string>
#include <vector>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/transport/Node.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
  /// \brief Plugin which checks if this model has touched some specific target
  /// for a given time continuously and exclusively. After the touch is
  /// completed, the plugin is disabled. It can be re-enabled through a Gazebo
  /// transport topic.
  ///
  /// It requires that contact sensors be placed in at least one link on this
  /// model.
  ///
  /// Parameters:
  ///
  /// <sensor> Name of contact sensor attached to one of this model's links.
  ///          There can be multiple sensor elements in case many links are
  ///          checked.
  ///
  /// <target> Scoped name of collision which we want to be touching. This can
  ///          be a substring of the desired collision name so we match more
  ///          than one collision. For example, using the name of a model will
  ///          match all its collisions.
  ///
  /// <time> Target time in seconds to maintain contact.
  ///
  /// <namespace> Namespace for transport topics:
  ///             /<namespace>/enable : Topic used to enable and disable the
  ///                                   plugin.
  ///             /<namespace>/touched : Topic where a message is published once
  ///                                    the touch is complete.
  ///
  /// <enabled> Set this to true so the plugin works from the start and doesn't
  ///           need to be enabled.
  ///
  class GAZEBO_VISIBLE TouchPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: TouchPlugin();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for enable "service".
    /// \param[in] _msg Message with 0 to disable and 1 to enable the plugin.
    public: void Enable(ConstIntPtr &_msg);

    /// \brief Update plugin's function.
    /// \_param[in] _info Information about world.
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Contact sensors attached to links in the model
    private: std::vector<sensors::ContactSensorPtr> contactSensors;

    /// \brief Name of this model, to be used to check collisions against
    private: std::string modelName;

    /// \brief Target collisions which this model should be touching. This can
    /// be a substring shared by several collisions.
    private: std::string target;

    /// \brief Namespace for transport topics.
    private: std::string ns;

    /// \brief Target time to continuously touch.
    private: common::Time targetTime = 5;

    /// \brief Time when started touching.
    private: common::Time touchStart;

    /// \brief Gazebo transport node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Publisher which publishes a message after touched for enough time
    private: transport::PublisherPtr touchedPub;

    /// \brief Subscriber to enable messages.
    private: transport::SubscriberPtr enableSub;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection;
  };
}

#endif

