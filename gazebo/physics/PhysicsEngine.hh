/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_PHYSICSENGINE_HH_
#define GAZEBO_PHYSICS_PHYSICSENGINE_HH_

#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <mutex>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class ContactManager;

    // Forward declare private data class
    class PhysicsEnginePrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class PhysicsEngine PhysicsEngine.hh physics/physics.hh
    /// \brief Base class for a physics engine.
    class GZ_PHYSICS_VISIBLE PhysicsEngine
    {
      /// \brief Default constructor.
      /// \param[in] _world Pointer to the world.
      public: explicit PhysicsEngine(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~PhysicsEngine();

      /// \brief Load the physics engine.
      /// \param[in] _sdf Pointer to the SDF parameters.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize the physics engine.
      public: virtual void Init() = 0;

      /// \brief Finilize the physics engine.
      public: virtual void Fini();

      /// \brief Rest the physics engine.
      public: virtual void Reset();

      /// \brief Init the engine for threads.
      public: virtual void InitForThread() = 0;

      /// \brief Update the physics engine collision.
      public: virtual void UpdateCollision() = 0;

      /// \brief Return the physics engine type (ode|bullet|dart|simbody).
      /// \return Type of the physics engine.
      /// \sa Type()
      public: virtual std::string GetType() const GAZEBO_DEPRECATED(7.0);

      /// \brief Return the physics engine type (ode|bullet|dart|simbody).
      /// \return Type of the physics engine.
      public: virtual std::string Type() const = 0;

      /// \brief Set the random number seed for the physics engine.
      /// \param[in] _seed The random number seed.
      public: virtual void SetSeed(const uint32_t _seed) = 0;

      /// \brief Get the simulation update period.
      /// \return Simulation update period.
      /// \deprecated See double UpdatePeriod() const
      public: double GetUpdatePeriod() GAZEBO_DEPRECATED(8.0);

      /// \brief Get the simulation update period.
      /// \return Simulation update period.
      public: double UpdatePeriod() const;

      /// \brief Get target real time factor
      /// \return Target real time factor
      /// \deprecated See double TargetRealTimeFactor() const
      public: double GetTargetRealTimeFactor() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get target real time factor
      /// \return Target real time factor
      public: double TargetRealTimeFactor() const;

      /// \brief Get real time update rate
      /// \return Update rate
      /// \deprecated See double RealTimeUpdateRate() const
      public: double GetRealTimeUpdateRate() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get real time update rate
      /// \return Update rate
      public: double RealTimeUpdateRate() const;

      /// \brief Get max step size.
      /// \return Max step size.
      public: double GetMaxStepSize() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get max step size.
      /// \return Max step size.
      public: double MaxStepSize() const;

      /// \brief Set target real time factor
      /// \param[in] _factor Target real time factor
      public: void SetTargetRealTimeFactor(const double _factor);

      /// \brief Set real time update rate
      /// \param[in] _rate Update rate
      public: void SetRealTimeUpdateRate(const double _rate);

      /// \brief Set max step size.
      /// \param[in] _stepSize Max step size.
      public: void SetMaxStepSize(const double _stepSize);

      /// \brief Update the physics engine.
      public: virtual void UpdatePhysics();

      /// \brief Create a new model.
      /// \param[in] _base Boost shared pointer to a new model.
      public: virtual ModelPtr CreateModel(BasePtr _base);

      /// \brief Create a new body.
      /// \param[in] _parent Parent model for the link.
      public: virtual LinkPtr CreateLink(ModelPtr _parent) = 0;

      /// \brief Create a collision.
      /// \param[in] _shapeType Type of collision to create.
      /// \param[in] _link Parent link.
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _link) = 0;

      /// \brief Create a collision.
      /// \param[in] _shapeType Type of collision to create.
      /// \param[in] _linkName Name of the parent link.
      public: CollisionPtr CreateCollision(const std::string &_shapeType,
                                           const std::string &_linkName);

      /// \brief Create a physics::Shape object.
      /// \param[in] _shapeType Type of shape to create.
      /// \param[in] _collision Collision parent.
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision) = 0;

      /// \brief Create a new joint.
      /// \param[in] _type Type of joint to create.
      /// \param[in] _parent Model parent.
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent = ModelPtr()) = 0;

      /// \brief Return the gravity vector.
      /// \return The gravity vector.
      /// \deprecated See ignition::math::Vector3d Gravity() const
      public: virtual math::Vector3 GetGravity() const GAZEBO_DEPRECATED(8.0);

      /// \brief Return the gravity vector.
      /// \return The gravity vector.
      public: virtual ignition::math::Vector3d Gravity() const;

      /// \brief Set the gravity vector.
      /// \param[in] _gravity New gravity vector.
      /// \deprecated See version that accepts ignition math parameters
      public: virtual void SetGravity(
                  const gazebo::math::Vector3 &_gravity) GAZEBO_DEPRECATED(7.0);

      /// \brief Set the gravity vector.
      /// \param[in] _gravity New gravity vector.
      public: virtual void SetGravity(
                  const ignition::math::Vector3d &_gravity) = 0;

      /// \brief Return the magnetic field vector.
      /// \return The magnetic field vector.
      public: virtual ignition::math::Vector3d MagneticField() const;

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief Access functions to set ODE parameters.
      /// \param[in] _autoDisable True to enable auto disabling of bodies.
      public: virtual void SetAutoDisableFlag(const bool _autoDisable);

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief access functions to set ODE parameters
      /// \param[in] _maxContacts Maximum number of contacts.
      public: virtual void SetMaxContacts(const unsigned int _maxContacts);

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief access functions to set ODE parameters..
      /// \return Auto disable flag.
      /// \deprecated See bool AutoDisableFlag()  const
      public: virtual bool GetAutoDisableFlag() GAZEBO_DEPRECATED(8.0);

      /// \TODO: Remove this function, and replace it with a more generic
      /// property map
      /// \brief access functions to set ODE parameters..
      /// \return Auto disable flag.
      public: virtual bool AutoDisableFlag() const;

      /// \brief Set a parameter of the physics engine.
      /// See SetParam documentation for descriptions of duplicate parameters.
      /// \param[in] _key String key
      /// Below is a list of _key parameter definitions:
      ///       -# "solver_type" (string) - returns solver used by engine, e.g.
      ///          "sequential_impulse' for Bullet, "quick" for ODE
      ///          "Featherstone and Lemkes" for DART and
      ///          "Spatial Algebra and Elastic Foundation" for Simbody.
      ///       -# "cfm" (double) - global CFM (ODE/Bullet)
      ///       -# "erp" (double) - global ERP (ODE/Bullet)
      ///       -# "precon_iters" (bool) - precondition iterations
      ///          (experimental). (ODE)
      ///       -# "iters" (int) - number of LCP PGS iterations. If
      ///          sor_lcp_tolerance is negative, full iteration count is
      ///          executed.  Otherwise, PGS may stop iteration early if
      ///          sor_lcp_tolerance is satisfied by the total RMS residual.
      ///       -# "sor" (double) - relaxation parameter for Projected
      ///          Gauss-Seidel (PGS) updates. (ODE/Bullet)
      ///       -# "contact_max_correcting_vel" (double) - truncates correction
      ///          impulses from ERP by this value. (ODE)
      ///       -# "contact_surface_layer" (double) - ERP is 0 for
      ///          interpenetration depths below this value. (ODE/Bullet)
      ///       -# "max_contacts" (int) - max number of contact constraints
      ///          between any pair of collision bodies.
      ///       -# "min_step_size" (double) - minimum internal step size.
      ///          (defined but not used in ode).
      ///       -# "max_step_size" (double) - maximum physics step size when
      ///          physics update step must return.
      ///
      /// \param[in] _value The value to set to
      /// \return true if SetParam is successful, false if operation fails.
      public: virtual bool SetParam(const std::string &_key,
                                    const boost::any &_value);

      /// \brief Get an parameter of the physics engine
      /// \param[in] _attr String key
      /// \sa SetParam
      /// \return The value of the parameter
      /// \deprecated See boost::any Param(const std::string &_key) const
      public: virtual boost::any GetParam(const std::string &_key) const
              GAZEBO_DEPRECATED(8.0);

      /// \brief Get an parameter of the physics engine
      /// \param[in] _attr String key
      /// \sa SetParam
      /// \return The value of the parameter
      public: virtual boost::any Param(const std::string &_key) const;

      /// \brief Get a parameter from the physics engine with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      /// \deprecated See bool Param(const std::string &_key,
      /// boost::any &_value) const
      public: virtual bool GetParam(const std::string &_key,
                  boost::any &_value) const GAZEBO_DEPRECATED(8.0);

      /// \brief Get a parameter from the physics engine with a boolean to
      /// indicate success or failure
      /// \param[in] _key Key of the accessed param
      /// \param[out] _value Value of the accessed param
      /// \return True if the parameter was successfully retrieved
      public: virtual bool Param(const std::string &_key,
                  boost::any &_value) const;

      /// \brief Debug print out of the physic engine state.
      public: virtual void DebugPrint() const = 0;

      /// \brief Get a pointer to the world.
      /// \return Pointer to the world.
      public: WorldPtr World() const;

      /// \brief Get a pointer to the contact manger.
      /// \return Pointer to the contact manager.
      /// \deprecated See ContactMgr() const
      public: ContactManager *GetContactManager() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get a pointer to the contact manger.
      /// \return Pointer to the contact manager.
      public: ContactManager *ContactMgr() const;

      /// \brief DO NOT USE. This function is here
      /// for backward compatibility when compiling. Use
      /// std::recursive_mutex &PhysicsUpdateMutex() const.
      /// \return NULL
      /// \deprecated See std::recursive_mutex &PhysicsUpdateMutex() const
      public: boost::recursive_mutex *GetPhysicsUpdateMutex() const
              GAZEBO_DEPRECATED(8.0);

      /// \brief Returns a reference to the PhysicsEngine#physicsUpdateMutex.
      /// \return Reference to the physics mutex.
      public: std::recursive_mutex &PhysicsUpdateMutex() const;

      /// \brief Get a pointer to the SDF element for this physics engine.
      /// \return Pointer to the physics SDF element.
      /// \deprecated See sdf::ElementPtr SDF() const;
      public: sdf::ElementPtr GetSDF() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get a pointer to the SDF element for this physics engine.
      /// \return Pointer to the physics SDF element.
      public: sdf::ElementPtr SDF() const;

      /// \internal
      /// \brief Constructor used by inherited classes.
      /// \param[in] _dataPtr Protected data class
      /// \param[in] _parent Pointer to the world.
      protected: PhysicsEngine(PhysicsEnginePrivate &_dataPtr, WorldPtr _world);

      /// \brief virtual callback for gztopic "~/request".
      /// \param[in] _msg Request message.
      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      /// \brief virtual callback for gztopic "~/physics".
      /// \param[in] _msg Physics message.
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Shared construction code.
      private: void ConstructionHelper();

      /// \internal
      /// \brief Private data pointer
      protected: PhysicsEnginePrivate *physicsEngineDPtr;
    };
    /// \}
  }
}
#endif
