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
#ifndef _GAZEBO_TRANSPORTER_PLUGIN_PRIVATE_HH_
#define _GAZEBO_TRANSPORTER_PLUGIN_PRIVATE_HH_

#include <map>
#include <string>
#include <mutex>
#include <memory>

#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>

namespace gazebo
{
  class TransporterPluginPrivate
  {
    /// \brief Definition of a transported pad
    public: class Pad
    {
      /// \brief Name of the pad.
      public: std::string name;

      /// \brief Name of the destination pad.
      public: std::string dest;

      /// \brief Pose of the incoming pad. This is where incoming models
      /// appear.
      public: math::Pose incomingPose;

      /// \brief Box that defines the activation region of the transporter.
      public: math::Box outgoingBox;

      /// \brief True if the pad should automatically teleport.
      /// False will cause the pad to wait for an activation
      /// signal. See this plugin's <activation_topic> xml element.
      public: bool autoActivation;

      /// \brief This flag is used for manual activation of a pad.
      /// It is set to true when a string message that contains
      /// the name of the pad is sent over the activation topic.
      public: bool activated;
    };

    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief Map of all the transporter pads
    public: std::map<std::string, std::shared_ptr<Pad> > pads;

    /// \brief Pointer to the update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Node for communication.
    public: transport::NodePtr node;

    /// \brief Subscriber to the activation topic.
    public: transport::SubscriberPtr activationSub;

    /// \brief Mutex to protect pad data.
    public: std::mutex padMutex;
  };
}
#endif
