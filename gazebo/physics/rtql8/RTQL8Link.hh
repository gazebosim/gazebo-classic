#ifndef _RTQL8LINK_HH_
#define _RTQL8LINK_HH_

#include "physics/rtql8/rtql8_inc.h"
#include "physics/rtql8/RTQL8Types.hh"
#include "physics/Link.hh"

//class BodyNode;

namespace gazebo
{
  namespace physics
  {
    //class BulletMotionState;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \brief rtql8 physics engine wrapper
    /// \{

    /// \brief RTQL8 Link class
    class RTQL8Link : public Link
    {
      /// \brief Constructor
      public: RTQL8Link(EntityPtr _parent);

      /// \brief Destructor
      public: virtual ~RTQL8Link();

      /// \brief Load the body based on an common::XMLConfig node
      public: virtual void Load(sdf::ElementPtr _ptr);

      /// \brief Initialize the body
      public: virtual void Init();

      /// \brief Finalize the body
      public: virtual void Fini();

      /// \brief Update the body
      public: virtual void Update();

	  
//       /// \brief Called when the pose of the entity (or one of its parents) has
//       /// changed
//       public: virtual void OnPoseChange();
// 
//       /// \brief Set whether this body is enabled
//       public: virtual void SetEnabled(bool enable) const;
// 
//       /// \brief Get whether this body is enabled in the physics engine
//       public: virtual bool GetEnabled() const {return true;}
// 
//       /// \brief Update the center of mass
//       public: virtual void UpdateCoM();
// 
//       /// \brief Set the linear velocity of the body
//       public: virtual void SetLinearVel(const math::Vector3 &vel);
// 
//       /// \brief Set the angular velocity of the body
//       public: virtual void SetAngularVel(const math::Vector3 &vel);
// 
//       /// \brief Set the force applied to the body
//       public: virtual void SetForce(const math::Vector3 &force);
// 
//       /// \brief Set the torque applied to the body
//       public: virtual void SetTorque(const math::Vector3 &force);
// 
//       /// \brief Get the linear velocity of the body in the world frame
//       public: virtual math::Vector3 GetWorldLinearVel() const;
// 
//       /// \brief Get the angular velocity of the body in the world frame
//       public: virtual math::Vector3 GetWorldAngularVel() const;
// 
//       /// \brief Get the force applied to the body in the world frame
//       public: virtual math::Vector3 GetWorldForce() const;
// 
//       /// \brief Get the torque applied to the body in the world frame
//       public: virtual math::Vector3 GetWorldTorque() const;
// 
//       /// \brief Set whether gravity affects this body
//       public: virtual void SetGravityMode(bool mode);
// 
//       /// \brief Get the gravity mode
//       public: virtual bool GetGravityMode();
// 
//       /// \brief Set whether this body will collide with others in the model
//       public: void SetSelfCollide(bool collide);
// 
//       /// \brief Get the bullet rigid body
//       public: btRigidBody *GetBulletLink() const;
// 
//       /// \brief Set the linear damping factor
//       public: virtual void SetLinearDamping(double damping);
// 
//       /// \brief Set the angular damping factor
//       public: virtual void SetAngularDamping(double damping);
// 
//       /// \brief Set the relative pose of a child collision.
//       /*public: void SetCollisionRelativePose(BulletCollision *collision,
//                                             const math::Pose &newPose);
//                                             */
// 
//       /// \brief Add a force to the body
//       public: virtual void AddForce(const math::Vector3 &_force);
// 
//       /// \brief Add a force to the body, components are relative to the
//       ///        body's own frame of reference.
//       public: virtual void AddRelativeForce(const math::Vector3 &_force);
// 
//       /// \brief Add a force to the body using a global position
//       public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
//                                                    const math::Vector3 &_pos);
// 
//       /// \brief Add a force to the body at position expressed to the body's
//       ///        own frame of reference.
//       public: virtual void AddForceAtRelativePosition(
//                   const math::Vector3 &_force,
//                   const math::Vector3 &_relpos);
// 
//       /// \brief Add a torque to the body
//       public: virtual void AddTorque(const math::Vector3 &_torque);
// 
//       /// \brief Add a torque to the body, components are relative to the
//       ///        body's own frame of reference.
//       public: virtual void AddRelativeTorque(const math::Vector3 &_torque);
// 
//       /// \copydoc Link::SetAutoDisable(bool)
//       public: virtual void SetAutoDisable(bool _disable);
// 
//       private: btCompoundShape *compoundShape;
//       private: BulletMotionState *motionState;
       private: kinematics::BodyNode* rtql8BodyNode;
       private: RTQL8PhysicsPtr rtql8Physics;
//       protected: math::Pose pose;
    };
    /// \}
  }
}
#endif
