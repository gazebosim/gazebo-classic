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
/* Desc: Body class
 * Author: Nate Koenig
 */

#ifndef BODY_HH
#define BODY_HH

#include <map>
#include <vector>

#include "common/Event.hh"
#include "common/CommonTypes.hh"

#include "physics/Entity.hh"
#include "physics/Inertial.hh"

namespace gazebo
{
	namespace physics
  {
    class Model;
    class Geom;
  
    /// \addtogroup gazebo_physics
    /// \brief The body class
    /// \{
  
    /// Body class
    class Body : public Entity
    {
      /// \brief Constructor
      public: Body(EntityPtr parent);
  
      /// \brief Destructor
      public: virtual ~Body();
  
      /// \brief Load the body based on an SDF element
      /// \param _sdf SDF parameters
      public: virtual void Load( sdf::ElementPtr &_sdf );
  
      /// \brief Initialize the body
      public: virtual void Init();
  
      /// \brief Finalize the body
      public: void Fini();
  
      /// \brief Update the body
      public: virtual void Update();
  
      /// \brief Set whether this body is enabled
      public: virtual void SetEnabled(bool enable) const = 0;
  
      /// \brief Get whether this body is enabled in the physics engine
      public: virtual bool GetEnabled() const = 0;
  
      /// \brief Set whether this entity has been selected by the user 
      ///        through the gui
      public: virtual bool SetSelected( bool s );
  
      /// \brief Set whether gravity affects this body
      public: virtual void SetGravityMode(bool mode) = 0;
  
      /// \brief Get the gravity mode
      public: virtual bool GetGravityMode() = 0;
  
  
      /// \brief Set whether this body will collide with others in the model
      public: virtual void SetSelfCollide(bool collide) = 0;
  
      /// \brief Set the collide mode of the body
      public: void SetCollideMode( const std::string &m );
  
      /// \brief Get Self-Collision Flag, if this is true, this body will collide
      //         with other bodies even if they share the same parent.
      public: bool GetSelfCollide();
  
     /// \brief Set the laser retro reflectiveness
      public: void SetLaserRetro(float retro);
  
      /// \brief Set the linear velocity of the body
      public: virtual void SetLinearVel(const math::Vector3 &vel) = 0;
  
      /// \brief Set the angular velocity of the body
      public: virtual void SetAngularVel(const math::Vector3 &vel) = 0;
  
      /// \brief Set the linear acceleration of the body
      public: void SetLinearAccel(const math::Vector3 &accel);
  
      /// \brief Set the angular acceleration of the body
      public: void SetAngularAccel(const math::Vector3 &accel);
  
      /// \brief Set the force applied to the body
      public: virtual void SetForce(const math::Vector3 &force) = 0;
  
      /// \brief Set the torque applied to the body
      public: virtual void SetTorque(const math::Vector3 &force) = 0;
  
  
      /// \brief Get the linear velocity of the body
      public: math::Vector3 GetRelativeLinearVel() const;
  
      /// \brief Get the angular velocity of the body
      public: math::Vector3 GetRelativeAngularVel() const;
  
      /// \brief Get the linear acceleration of the body
      public: math::Vector3 GetRelativeLinearAccel() const;
  
      /// \brief Get the linear acceleration of the body in the world frame
      public: math::Vector3 GetWorldLinearAccel() const;
  
      /// \brief Get the angular acceleration of the body
      public: math::Vector3 GetRelativeAngularAccel() const;
  
      /// \brief Get the angular acceleration of the body in the world frame
      public: math::Vector3 GetWorldAngularAccel() const;
  
      /// \brief Get the force applied to the body
      public: math::Vector3 GetRelativeForce() const;
  
      /// \brief Get the force applied to the body in the world frame
      public: virtual math::Vector3 GetWorldForce() const = 0;
  
      /// \brief Get the torque applied to the body
      public: math::Vector3 GetRelativeTorque() const;
  
      /// \brief Get the torque applied to the body in the world frame
      public: virtual math::Vector3 GetWorldTorque() const = 0;
  
  
      /// \brief Get the model that this body belongs to
      public: ModelPtr GetModel() const;
  
      /// \brief Get the mass of the body
      public: InertialPtr GetInertial() const { return this->inertial; }
  
      /// \brief Set the mass of the body
      public: void SetInertial(const InertialPtr &_inertial);
  
      /// Load a new geom helper function
      /// \param _sdf SDF element used to load the geom
      private: void LoadGeom( sdf::ElementPtr &_sdf );
  
      /// \brief Load a renderable
      private: void LoadVisual( sdf::ElementPtr &_sdf );
  
      /// \brief  Get the size of the body
      public: virtual math::Box GetBoundingBox() const;
  
      /// \brief Set the linear damping factor
      public: virtual void SetLinearDamping(double damping) = 0;
  
      /// \brief Set the angular damping factor
      public: virtual void SetAngularDamping(double damping) = 0;
  
      /// \brief Set whether this body is in the kinematic state
      public: virtual void SetKinematic(const bool &) {}
  
      /// \brief Get whether this body is in the kinematic state
      public: virtual bool GetKinematic() const {return false;}
  
      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectEnabledSignal( T subscriber )
              { return enabledSignal.Connect(subscriber); }
  
      public: void DisconnectEnabledSignal( event::ConnectionPtr &c )
              { enabledSignal.Disconnect(c); }
  
      protected: bool isStatic;
  
      /// Used by Model if this body is the canonical body
      ///   model pose = body pose + initModelOffset
      public: math::Pose initModelOffset;
  
      // Helper entity for separating body pose from center-of-mass pose
      protected: EntityPtr comEntity;
  
      /// The pose of the body relative to the model. Can also think of this
      /// as the body's pose offset.
      protected: math::Pose relativePose;
 
      protected: InertialPtr inertial;

      protected: std::vector< std::string > cgVisuals;
  
      protected: math::Vector3 linearAccel;
      protected: math::Vector3 angularAccel;
  
      protected: std::vector<std::string> visuals;
  
      private: event::EventT<void (bool)> enabledSignal;
      private: event::ConnectionPtr showPhysicsConnection; 
  
      /// This flag is used to trigger the enabledSignal
      private: bool enabled;
  
      protected: math::Pose newPose;
  
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
