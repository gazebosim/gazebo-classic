/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: Camera Visualization Class
 * Author: Nate Koenig
 */

#ifndef _CONTACTVISUAL_HH_
#define _CONTACTVISUAL_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace Ogre
{
  class Entity;
  class SceneNode;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    class DynamicLines;

    /// \class ContactVisual ContactVisual.hh rendering/rendering.hh
    /// \brief Contact visualization
    ///
    /// This class visualizes contact points by drawing arrows in the 3D
    /// environment.
    class ContactVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the ContactVisual
      /// \param[in] _vis Pointer the parent Visual
      /// \param[in] _topicName Name of the topic which publishes the contact
      /// information.
      public: ContactVisual(const std::string &_name, VisualPtr _vis,
                            const std::string &_topicName);

      /// \brief Destructor
      public: virtual ~ContactVisual();

      /// \brief Set to true to enable contact visualization.
      /// \param[in] _enabled True to show contacts, false to hide.
      public: void SetEnabled(bool _enabled);

      /// \brief Update the Visual
      private: void Update();

      /// \brief Callback when a Contact message is received
      /// \param[in] _msg The Contact message
      private: void OnContact(ConstContactsPtr &_msg);

      /// \brief Create a new contact visualization point.
      private: void CreateNewPoint();

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// \brief Subscription to the contact data.
      private: transport::SubscriberPtr contactsSub;

      /// \brief The current contact message.
      private: boost::shared_ptr<msgs::Contacts const> contactsMsg;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief A contact point visualization.
      private: class ContactPoint
               {
                 /// \brief The scene node for the contact visualization.
                 public: Ogre::SceneNode *sceneNode;
                 /// \brief Normal and depth for the contact point.
                 public: DynamicLines *normal, *depth;
               };

      /// \brief All the contact points.
      private: std::vector<ContactVisual::ContactPoint*> points;

      /// \brief Mutex to protect the contact message.
      private: boost::mutex mutex;

      /// \brief True if we have received a message.
      private: bool receivedMsg;

      /// \brief True if this visualization is enabled.
      private: bool enabled;

      /// \brief Name of the topic contact information is published on
      private: std::string topicName;
    };
    /// \}
  }
}
#endif
