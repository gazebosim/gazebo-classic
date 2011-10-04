/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef COLLISION_HH
#define COLLISION_HH

#include "common/Event.hh"
#include "common/CommonTypes.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/Entity.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for all collision entities
    class Collision : public Entity
    {
      /// \brief Constructor
      public: Collision(LinkPtr link);
    
      /// \brief Destructor
      public: virtual ~Collision();
  
      /// \brief Finalize the collision
      public: void Fini();
  
      /// \brief Load the collision
      public: virtual void Load( sdf::ElementPtr &_sdf );

      public: virtual void Init();

      /// \brief Update the parameters using new sdf values
      public: virtual void UpdateParameters( sdf::ElementPtr &_sdf );

      /// \brief Load the collision
      public: void Save(std::string &prefix, std::ostream &stream);
   
      /// \brief Set the encapsulated collsion object
      public: void SetCollision(bool placeable);
    
      /// \brief Return whether this collision is placeable
      public: bool IsPlaceable() const;
      
      /// \brief Set the category bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCategoryBits(unsigned int bits) = 0;
    
      /// \brief Set the collide bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCollideBits(unsigned int bits) = 0;
    
      /// \brief Set the laser retro reflectiveness 
      public: void SetLaserRetro(float retro);
    
      /// \brief Get the laser retro reflectiveness 
      public: float GetLaserRetro() const;
  
      /// \brief Set the visibility of the bounding box
      public: void ShowBoundingBox(const bool &show);
  
      /// \brief Get the link this collision belongs to
      public: LinkPtr GetLink() const;
  
      /// \brief Get the model this collision belongs to
      public: ModelPtr GetModel() const;
  
      /// \brief Get the bounding box for this collision
      public: virtual math::Box GetBoundingBox() const = 0;
  
      /// \brief Get the shape type
      public: Base::EntityType GetShapeType();
  
      /// \brief Set the shape for this collision
      public: void SetShape(ShapePtr shape);
              
      /// \brief Get the attached shape
      public: ShapePtr GetShape() const;
  
      /// \brief Turn contact recording on or off
      public: void SetContactsEnabled(const bool &enable);
  
      /// \brief Return true of contact recording is on
      public: bool GetContactsEnabled() const;
  
      /// \brief Add an occurance of a contact to this collision
      public: void AddContact(const Contact &contact);
  
      /// \brief Clear all contact info
      public: void ClearContacts();
  
      /// \brief Get the number of contacts
      public: unsigned int GetContactCount() const;
              
      /// \brief Get a specific contact
      public: Contact GetContact(unsigned int i);
  
      /// \brief Get the linear velocity of the collision
      public: virtual math::Vector3 GetRelativeLinearVel() const;
  
      /// \brief Get the linear velocity of the collision in the world frame
      public: virtual math::Vector3 GetWorldLinearVel() const;
  
      /// \brief Get the angular velocity of the collision
      public: virtual math::Vector3 GetRelativeAngularVel() const;
  
      /// \brief Get the angular velocity of the collision in the world frame
      public: virtual math::Vector3 GetWorldAngularVel() const;
  
      /// \brief Get the linear acceleration of the collision
      public: virtual math::Vector3 GetRelativeLinearAccel() const;
              
      /// \brief Get the linear acceleration of the collision in the world frame
      public: virtual math::Vector3 GetWorldLinearAccel() const;
  
      /// \brief Get the angular acceleration of the collision
      public: virtual math::Vector3 GetRelativeAngularAccel() const;
  
      /// \brief Get the angular acceleration of the collision in the world frame
      public: virtual math::Vector3 GetWorldAngularAccel() const;
  
      public: template< typename T>
              event::ConnectionPtr ConnectContactCallback( T subscriber )
              { return contactSignal.Connect(subscriber); }
      public: void DisconnectContactCallback( event::ConnectionPtr &c )
              { contactSignal.Disconnect(c); }

      /// \brief Fill a collision message
      public: void FillCollisionMsg( msgs::Collision &_msg );
  
      /// \brief Enable callback: Called when the link changes
      private: void EnabledCB(bool enabled);
  
      /// \brief Create the bounding box for the collision
      private: void CreateBoundingBox();
  
 
      /// The link this collision belongs to
      protected: LinkPtr link;
    
      protected: bool placeable;
  
      private: float transparency;
  
      /// All the visual apparence 
      private: std::string bbVisual;
  
      protected: ShapePtr shape;

      private: bool contactsEnabled;
  
      public: event::EventT< void (const Contact &)> contactSignal;
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
