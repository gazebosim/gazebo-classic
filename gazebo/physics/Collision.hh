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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef _COLLISION_HH_
#define _COLLISION_HH_

#include <string>
#include <vector>

#include "common/Event.hh"
#include "common/CommonTypes.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/CollisionState.hh"
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
      public: Collision(LinkPtr _link);

      /// \brief Destructor
      public: virtual ~Collision();

      /// \brief Finalize the collision
      public: void Fini();

      /// \brief Load the collision
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the collision
      public: virtual void Init();

      /// \brief Update the parameters using new sdf values
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Set the encapsulated collsion object
      public: void SetCollision(bool _placeable);

      /// \brief Return whether this collision is placeable
      public: bool IsPlaceable() const;

      /// \brief Set the category bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCategoryBits(unsigned int _bits) = 0;

      /// \brief Set the collide bits, used during collision detection
      /// \param bits The bits
      public: virtual void SetCollideBits(unsigned int _bits) = 0;

      /// \brief Set the laser retro reflectiveness
      public: void SetLaserRetro(float _retro);

      /// \brief Get the laser retro reflectiveness
      public: float GetLaserRetro() const;

      /// \brief Get the link this collision belongs to
      public: LinkPtr GetLink() const;

      /// \brief Get the model this collision belongs to
      public: ModelPtr GetModel() const;

      /// \brief Get the bounding box for this collision
      public: virtual math::Box GetBoundingBox() const = 0;

      /// \brief Get the shape type
      public: unsigned int GetShapeType();

      /// \brief Set the shape for this collision
      public: void SetShape(ShapePtr _shape);

      /// \brief Get the attached shape
      public: ShapePtr GetShape() const;

      /// \brief Turn contact recording on or off
      public: void SetContactsEnabled(bool _enable);

      /// \brief Return true of contact recording is on
      public: bool GetContactsEnabled() const;

      /// \brief Add an occurance of a contact to this collision
      public: void AddContact(const Contact &_contact);

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

      /// \brief Get the angular acceleration of the collision in the
      ///        world frame
      public: virtual math::Vector3 GetWorldAngularAccel() const;

      /// \brief Get the collision state
      public: CollisionState GetState();

      /// \brief Set the current collision state
      public: void SetState(const CollisionState &_state);

      /// \brief Setup callback for contact event
      public: template<typename T>
              event::ConnectionPtr ConnectContact(T _subscriber)
              {return contact.Connect(_subscriber);}

      /// \brief Disconnect callback for contact event
      public: void DisconnectContact(event::ConnectionPtr &_c)
              {contact.Disconnect(_c);}

      /// \brief Fill a collision message
      public: void FillCollisionMsg(msgs::Collision &_msg);

      /// \brief Update parameters from a message
      public: void ProcessMsg(const msgs::Collision &_msg);

      /// \brief Get the surface parameters
      public: inline SurfaceParamsPtr GetSurface() const
              {return this->surface;}

      private: msgs::Visual CreateCollisionVisual();

      /// The link this collision belongs to
      protected: LinkPtr link;

      /// flag for placeable
      protected: bool placeable;

      private: float transparency;

      /// All the visual apparence
      private: std::string bbVisual;

      /// pointer to physics::Shape
      protected: ShapePtr shape;

      private: bool contactsEnabled;

      /// contact event
      public: event::EventT<void (const std::string &,
                                  const Contact &)> contact;

      private: SurfaceParamsPtr surface;
      private: std::vector<event::ConnectionPtr> connections;

      private: float laserRetro;
    };
    /// \}
  }
}
#endif
