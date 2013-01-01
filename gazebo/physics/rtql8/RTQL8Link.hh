#ifndef _RTQL8LINK_HH_
#define _RTQL8LINK_HH_

#include "physics/Link.hh"

#include "physics/rtql8/rtql8_inc.h"
#include "physics/rtql8/RTQL8Types.hh"

namespace gazebo
{
  namespace physics
  {
    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_rtql8 RTQL8 Physics
    /// \brief rtql8 physics engine wrapper
    /// \{

    /// \brief RTQL8 Link class
    class RTQL8Link : public Link
    {
      /// \brief Constructor
      public: explicit RTQL8Link(EntityPtr _parent);

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
	  
	  // Documentation inherited
      public: virtual void OnPoseChange();

      // Documentation inherited
      public: virtual void SetEnabled(bool _enable) const;

      // Documentation inherited
      public: virtual bool GetEnabled() const;

      // Documentation inherited
      public: virtual void UpdateMass();

      // Documentation inherited
      public: virtual void UpdateSurface();

      // Documentation inherited
      public: virtual void SetLinearVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      // Documentation inherited
      public: virtual void SetForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void SetTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      // Documentation inherited
      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      // Documentation inherited
      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relpos);

      // Documentation inherited
      public: virtual void AddTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      // Documentation inherited
      public: virtual math::Vector3 GetWorldLinearVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldAngularVel() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldForce() const;

      // Documentation inherited
      public: virtual math::Vector3 GetWorldTorque() const;

      // Documentation inherited
      public: virtual void SetGravityMode(bool _mode);

      // Documentation inherited
      public: virtual bool GetGravityMode();

      // Documentation inherited
      public: void SetSelfCollide(bool _collide);

      // Documentation inherited
      public: virtual void SetLinearDamping(double _damping);

      // Documentation inherited
      public: virtual void SetAngularDamping(double _damping);

      // Documentation inherited
      public: virtual void SetKinematic(const bool &_state);

      // Documentation inherited
      public: virtual bool GetKinematic() const;

      // Documentation inherited
      public: virtual void SetAutoDisable(bool _disable);

      /// \brief
      public: kinematics::BodyNode* GetBodyNode() const {return rtql8BodyNode;}

      /// \brief
      private: kinematics::BodyNode* rtql8BodyNode;
	  
      /// \brief
      private: RTQL8PhysicsPtr rtql8Physics;

    };
    /// \}
  }
}
#endif
