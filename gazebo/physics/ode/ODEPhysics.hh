/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _ODEPHYSICS_HH_
#define _ODEPHYSICS_HH_

#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>
#include <string>
#include <utility>

#include <boost/thread/thread.hpp>

#include "gazebo/physics/ode/ode_inc.h"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class ODEJointFeedback;
    class ODEPhysicsPrivate;

    /// \brief ODE physics engine.
    class GZ_PHYSICS_VISIBLE ODEPhysics : public PhysicsEngine
    {
      /// \enum ODEParam
      /// \brief ODE Physics parameter types.
      public: enum ODEParam
      {
        /// \brief Solve type
        SOLVER_TYPE,

        /// \brief Constraint force mixing
        GLOBAL_CFM,

        /// \brief Error reduction parameter
        GLOBAL_ERP,

        /// \brief Number of iterations
        SOR_PRECON_ITERS,

        /// \brief Number of iterations
        PGS_ITERS,

        /// \brief SOR over-relaxation parameter
        SOR,

        /// \brief Max correcting velocity
        CONTACT_MAX_CORRECTING_VEL,

        /// \brief Surface layer depth
        CONTACT_SURFACE_LAYER,

        /// \brief Maximum number of contacts
        MAX_CONTACTS,

        /// \brief Minimum step size
        MIN_STEP_SIZE,

        /// \brief Limit ratios of inertias of adjacent links (note that the
        /// corresponding SDF tag is "use_dynamic_moi_rescaling")
        INERTIA_RATIO_REDUCTION,

        /// \brief friction model
        FRICTION_MODEL,

        /// \brief LCP Solver
        WORLD_SOLVER_TYPE
      };

      /// \brief Constructor.
      /// \param[in] _world The World that uses this physics engine.
      public: ODEPhysics(WorldPtr _world);

      /// \brief Destructor.
      public: virtual ~ODEPhysics();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Reset();

      // Documentation inherited
      public: virtual void InitForThread();

      // Documentation inherited
      public: virtual void UpdateCollision();

      // Documentation inherited
      public: virtual void UpdatePhysics();

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetType() const
                      { return "ode"; }

      // Documentation inherited
      public: virtual LinkPtr CreateLink(ModelPtr _parent);

      // Documentation inherited
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _parent);

      // Documentation inherited
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);

      // Documentation inherited
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent);

      // Documentation inherited
      public: virtual void SetGravity(const gazebo::math::Vector3 &_gravity);

      /// \brief Set the CFM parameter
      /// \param[in] _cfm New constraint force mixing  value
      /// \sa GetWorldCFM
      public: void SetWorldCFM(const double &_cfm);

      /// \brief Set the ERP parameter
      /// \param[in] _value New error reduction parameter value
      /// \sa GetWorldERP
      public: void SetWorldERP(double erp);

      /// \brief Get the minimum step size
      /// \return The minimum step size.
      /// \sa SetMinStepSize
      public: double MinStepSize() const;

      /// \brief Set the minimum step size
      /// \param[in] _value The minimum step size.
      /// \sa MinStepSize
      public: void SetMinStepSize(const double &_value);

      // Documentation inherited
      public: virtual void SetSORPGSPreconIters(unsigned int iters);

      /// \brief Set the successive over-relaxation PGS iterations parameter
      /// value.
      /// \param[in] _iters The successive over-relaxation PGS iterations.
      /// value.
      /// \sa GetSORPGSIters
      public: void SetSORPGSIters(const unsigned int &_iters);

      // Documentation inherited
      public: virtual void SetSORPGSW(double w);

      /// \brief Set the contact max correcting velocity parameter value.
      /// \param[in] _vel Max correcting velocity.
      /// \sa SetContactMaxCorrectingVel
      public: void SetContactMaxCorrectingVel(const double &_vel);

      /// \brief Set the contact surface layer parameter value
      /// \param[in] _layerDepth Surface layer depth
      /// \sa GetContactSurfaceLayer
      public: void SetContactSurfaceLayer(const double &_layerDepth);

      /// \brief Set friction model type.
      /// \param[in] _fricModel Type of friction model.
      public: virtual void SetFrictionModel(const std::string &_fricModel);

      /// \brief Set world step solver type.
      /// \param[in] _worldSolverType Type of solver used by world step.
      public: virtual void
              SetWorldStepSolverType(const std::string &_worldSolverType);

      // Documentation inherited
      public: virtual void SetMaxContacts(const unsigned int &_maxContacts);

      /// \brief Get the CFM parameter
      /// \return The constraint force mixing parameter value
      /// \sa SetWorldCFM
      public: double GetWorldCFM() const;

      /// \brief Get the ERP parameter
      /// \return The error reduction parameter value
      /// \sa SetWorldERP
      public: double GetWorldERP();

      // Documentation inherited
      public: virtual int GetSORPGSPreconIters();

      /// \brief Get the successive over-relaxation PGS iterations parameter
      /// value.
      /// \return The successive over-relaxation PGS iterations parameter
      /// value.
      /// \sa SetSORPGSIters
      public: unsigned int GetSORPGSIters() const;

      // Documentation inherited
      public: virtual double GetSORPGSW();

      /// \brief Get the contact max correcting velocity parameter value.
      /// \return Max correcting velocity value.
      /// \sa SetContactMaxCorrectingVel
      public: double GetContactMaxCorrectingVel() const;

      /// \brief Get friction model.
      /// \return Friction model type.
      public: virtual std::string GetFrictionModel() const;

      /// \brief Get solver type for world step.
      /// \return Type of solver used by world step.
      public: virtual std::string GetWorldStepSolverType() const;

      /// \brief Get the contact surface layer parameter value
      /// \return Contact suerface layer depth.
      /// \sa SetContactSurfaceLayer
      public: double GetContactSurfaceLayer() const;

      // Documentation inherited
      public: virtual unsigned int GetMaxContacts();

      // Documentation inherited
      public: virtual void DebugPrint() const;

      // Documentation inherited
      public: virtual void SetSeed(uint32_t _seed);

      /// Documentation inherited
      public: virtual boost::any GetParam(const std::string &_key) const;

      /// Documentation inherited
      public: virtual bool GetParam(const std::string &_key,
                  boost::any &_value) const;

      /// \brief Return the world space id.
      /// \return The space id for the world.
      public: dSpaceID GetSpaceId() const;

      /// \brief Get the world id.
      /// \return The world id.
      public: dWorldID GetWorldId();

      /// \brief Convert an ODE mass to Inertial.
      /// \param[out] _intertial Pointer to an Inertial object.
      /// \param[in] _odeMass Pointer to an ODE mass that will be converted.
      public: static void ConvertMass(InertialPtr _interial, void *_odeMass);

      /// \brief Convert an Inertial to ODE mass.
      /// \param[out] _odeMass Pointer to an ODE mass.
      /// \param[in] _intertial Pointer to an Inertial object that will be
      /// converted.
      public: static void ConvertMass(void *_odeMass, InertialPtr _inertial);

      /// \brief Convert a string to a Friction_Model enum.
      /// \param[in] _fricModel Friction model string.
      /// \return A Friction_Model enum. Defaults to pyramid_friction
      /// if _fricModel is unrecognized.
      public: static Friction_Model
              ConvertFrictionModel(const std::string &_fricModel);

      /// \brief Convert a Friction_Model enum to a string.
      /// \param[in] _fricModel Friction_Model enum.
      /// \return Friction model string. Returns "unknown" if
      /// _fricModel is unrecognized.
      public: static std::string
              ConvertFrictionModel(const Friction_Model _fricModel);

      /// \brief Convert a World_Solver_Type enum to a string.
      /// \param[in] _solverType World_Solver_Type enum.
      /// \return world solver type string. Returns "unknown" if
      /// _solverType is unrecognized.
      public: static std::string
              ConvertWorldStepSolverType(const World_Solver_Type _solverType);

      /// \brief Convert a string to a World_Solver_Type enum.
      /// \param[in] _solverType world solver type string.
      /// \return A World_Solver_Type enum. Defaults to ODE_DEFAULT
      /// if _solverType is unrecognized.
      public: static World_Solver_Type
              ConvertWorldStepSolverType(const std::string &_solverType);

      /// \brief Get the step type (quick, world).
      /// \return The step type.
      public: std::string GetStepType() const;

      /// \brief Set the step type (quick, world).
      /// \param[in] _type The step type (quick or world).
      public: virtual void SetStepType(const std::string &_type);


      /// \brief Collide two collision objects.
      /// \param[in] _collision1 First collision object.
      /// \param[in] _collision2 Second collision object.
      /// \param[in,out] _contactCollision Array of contacts.
      public: void Collide(ODECollision *_collision1, ODECollision *_collision2,
                           dContactGeom *_contactCollisions);

      /// \brief process joint feedbacks.
      /// \param[in] _feedback ODE Joint Contact feedback information.
      public: void ProcessJointFeedback(ODEJointFeedback *_feedback);

      protected: virtual void OnRequest(ConstRequestPtr &_msg);

      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      /// \brief Primary collision callback.
      /// \param[in] _data Pointer to user data.
      /// \param[in] _o1 First geom to check for collisions.
      /// \param[in] _o2 Second geom to check for collisions.
      private: static void CollisionCallback(void *_data, dGeomID _o1,
                                             dGeomID _o2);


      /// \brief Create a triangle mesh object collider.
      /// \param[in] _collision1 The first collision object.
      /// \param[in] _collision2 The second collision object.
      private: void AddTrimeshCollider(ODECollision *_collision1,
                                       ODECollision *_collision2);

      /// \brief Create a normal object collider.
      /// \param[in] _collision1 The first collision object.
      /// \param[in] _collision2 The second collision object.
      private: void AddCollider(ODECollision *_collision1,
                                ODECollision *_collision2);


      /// \brief Get the SOR LCP Tolerance value
      /// \return The successive over-relaxation linear complementatory
      /// problem tolerance value.
      /// \sa SetSORLCPTolerance
      public: double SORLCPTolerance() const;

      /// \brief Set the SOR LCP Tolerance value
      /// \param[in] _value The successive over-relaxation linear
      /// complementatory problem tolerance value.
      /// \sa SORLCPTolerance
      public: void SetSORLCPTolerance(const double &_value);

      /// \brief Get whether dynamic rescaling of the momement of inertia is
      /// enabled.
      /// \return True if dynamic rescaling of the MOI is enabled.
      /// \sa SetUseDynamicMOIRescaling
      public: bool UseDynamicMOIRescaling() const;

      /// \brief Set whether dynamic rescaling of the momement of inertia is
      /// enabled.
      /// \param[in] _value True if dynamic rescaling of the MOI is enabled.
      /// \sa UseDynamicMOIRescaling
      public: void SetUseDynamicMOIRescaling(const bool &_value);

      /// \brief Get the contact residual smoothing parameter value.
      /// \return the contact residual smoothing value
      /// \sa SetContactResidualSmoothing
      public: double ContactResidualSmoothing() const;

      /// \brief Set the contact residual smoothing parameter value.
      /// \param[in] _value The contact residual smoothing value
      /// \sa ContactResidualSmoothing
      public: void SetContactResidualSmoothing(const double &_value);

      /// \brief Get the thread position correction parameter value.
      /// \return The thread position correction parameter value
      /// \sa SetThreadPositionCorrection
      public: bool ThreadPositionCorrection() const;

      /// \brief Set the thread position correction parameter value.
      /// \param[in] _value The thread position correction parameter value
      /// \sa ThreadPositionCorrection
      public: void SetThreadPositionCorrection(const bool &_value);

      /// \brief Get the experimental row reordering parameter value
      /// \return The experimental row reordering parameter value
      /// \sa SetExperimentalRowReordering
      public: bool ExperimentalRowReordering() const;

      /// \brief Set the experimental row reordering parameter value
      /// \param[in] _value The experimental row reordering parameter value
      /// \sa ExperimentalRowReordering
      public: void SetExperimentalRowReordering(const bool &_value);

      /// \brief Get the warm start factor
      /// \return The warm start factor
      /// \sa SetWarmStartFactor
      public: double WarmStartFactor() const;

      /// \brief Set the warm start factor
      /// \param[in] _value The warm start factor
      /// \sa WarmStartFactor
      public: void SetWarmStartFactor(const double &_value);

      /// \brief Get the extra friction iterations parameter value
      /// \return The extra friction iterations parameter value
      /// \sa SetExtraFrictionIterations
      public: double ExtraFrictionIterations() const;

      /// \brief Get the extra friction iterations parameter value
      /// \param[in] _value The extra friction iterations parameter value
      /// \sa ExtraFrictionIterations
      public: void SetExtraFrictionIterations(const int &_value);

      /// \brief Helper function to create all the ODE parameters.
      private: void CreateParams();

      /// \internal
      /// \brief Private data pointer.
      private: ODEPhysicsPrivate *dataPtr;
    };
  }
}
#endif
