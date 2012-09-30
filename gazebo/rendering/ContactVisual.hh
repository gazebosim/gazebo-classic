/*
 * Copyright 2011 Nate Koenig
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

#ifndef CONTACTVISUAL_HH
#define CONTACTVISUAL_HH

#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
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

    /// \class ContactVisual ContactVisual.hh rendering/ContactVisual.hh
    /// \brief Contact visualization
    ///
    /// This class visualizes contact points by drawing arrows in the 3D
    /// environment.
    class ContactVisual : public Visual
    {
      /// \brief Constructor
      /// \param _name Name of the ContactVisual
      /// \param _vis Pointer the parent Visual
      /// \arapm _topicName Name of the topic which publishes the contact
      /// information.
      public: ContactVisual(const std::string &_name, VisualPtr _vis,
                            const std::string &_topicName);

      /// \brief Destructor
      public: virtual ~ContactVisual();

      /// \brief Update the Visual
      private: void Update();

      /// \brief Callback when a Contact message is received
      /// \param _msg The Contact message
      private: void OnContact(ConstContactsPtr &_msg);

      private: void SetupInstancedMaterialToEntity(Ogre::Entity *_ent);

      private: transport::NodePtr node;
      private: transport::SubscriberPtr contactsSub;
      private: boost::shared_ptr<msgs::Contacts const> contactsMsg;
      private: std::vector<event::ConnectionPtr> connections;

      private: class ContactPoint
               {
                 public: Ogre::SceneNode *sceneNode;
                 public: DynamicLines *normal, *depth;
               };
      private: std::vector<ContactVisual::ContactPoint*> points;
    };
    /// \}
  }
}
#endif


