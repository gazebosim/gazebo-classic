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

#include <string>

#include "gazebo/physics/Inertial.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyModel.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyJoint.hh"
#include "gazebo/physics/simbody/SimbodyCollision.hh"

#include "gazebo/physics/simbody/SimbodyPlaneShape.hh"
#include "gazebo/physics/simbody/SimbodySphereShape.hh"
#include "gazebo/physics/simbody/SimbodyHeightmapShape.hh"
#include "gazebo/physics/simbody/SimbodyMultiRayShape.hh"
#include "gazebo/physics/simbody/SimbodyBoxShape.hh"
#include "gazebo/physics/simbody/SimbodyCylinderShape.hh"
#include "gazebo/physics/simbody/SimbodyMeshShape.hh"
#include "gazebo/physics/simbody/SimbodyPolylineShape.hh"
#include "gazebo/physics/simbody/SimbodyRayShape.hh"

#include "gazebo/physics/simbody/SimbodyHingeJoint.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"
#include "gazebo/physics/simbody/SimbodyBallJoint.hh"
#include "gazebo/physics/simbody/SimbodySliderJoint.hh"
#include "gazebo/physics/simbody/SimbodyHinge2Joint.hh"
#include "gazebo/physics/simbody/SimbodyScrewJoint.hh"
#include "gazebo/physics/simbody/SimbodyFixedJoint.hh"

#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/MapShape.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/simbody/SimbodyPhysicsPrivate.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"

using namespace gazebo;
using namespace physics;
using namespace SimTK;

GZ_REGISTER_PHYSICS_ENGINE("simbody", SimbodyPhysics)

//////////////////////////////////////////////////
SimbodyPhysics::SimbodyPhysics(WorldPtr _world)
: PhysicsEngine(_world),
  simbodyPhysicsDPtr(new SimbodyPhysicsPrivate)
{
  // Instantiate the Multibody System
  // Instantiate the Simbody Matter Subsystem
  // Instantiate the Simbody General Force Subsystem

  this->simbodyPhysicsDPtr->simbodyPhysicsInitialized = false;
  this->simbodyPhysicsDPtr->simbodyPhysicsStepped = false;
}

//////////////////////////////////////////////////
SimbodyPhysics::~SimbodyPhysics()
{
}

//////////////////////////////////////////////////
ModelPtr SimbodyPhysics::CreateModel(BasePtr _parent)
{
  // set physics as uninitialized
  this->simbodyPhysicsDPtr->simbodyPhysicsInitialized = false;

  SimbodyModelPtr model(new SimbodyModel(_parent));

  return model;
}

//////////////////////////////////////////////////
void SimbodyPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Create an integrator
  /// \TODO: get from sdf for simbody physics
  /// \TODO: use this when pgs rigid body solver is implemented
  this->simbodyPhysicsDPtr->solverType = "elastic_foundation";

  /// \TODO: get from sdf for simbody physics
  this->simbodyPhysicsDPtr->integratorType = "semi_explicit_euler";

  if (this->simbodyPhysicsDPtr->integratorType == "rk_merson")
  {
    this->simbodyPhysicsDPtr->integ =
      new SimTK::RungeKuttaMersonIntegrator(this->simbodyPhysicsDPtr->system);
  }
  else if (this->simbodyPhysicsDPtr->integratorType == "rk3")
  {
    this->simbodyPhysicsDPtr->integ = new SimTK::RungeKutta3Integrator(
        this->simbodyPhysicsDPtr->system);
  }
  else if (this->simbodyPhysicsDPtr->integratorType == "rk2")
  {
    this->simbodyPhysicsDPtr->integ = new SimTK::RungeKutta2Integrator(
        this->simbodyPhysicsDPtr->system);
  }
  else if (this->simbodyPhysicsDPtr->integratorType == "semi_explicit_euler")
  {
    this->simbodyPhysicsDPtr->integ =
      new SimTK::SemiExplicitEuler2Integrator(this->simbodyPhysicsDPtr->system);
  }
  else
  {
    gzerr << "type not specified, using SemiExplicitEuler2Integrator.\n";
    this->simbodyPhysicsDPtr->integ =
      new SimTK::SemiExplicitEuler2Integrator(this->simbodyPhysicsDPtr->system);
  }

  this->simbodyPhysicsDPtr->stepTimeDouble = this->MaxStepSize();

  sdf::ElementPtr simbodyElem =
    this->simbodyPhysicsDPtr->sdf->GetElement("simbody");

  // Set integrator accuracy (measured with Richardson Extrapolation)
  this->simbodyPhysicsDPtr->integ->setAccuracy(
    simbodyElem->Get<double>("accuracy"));

  // Set stiction max slip velocity to make it less stiff.
  this->simbodyPhysicsDPtr->contact.setTransitionVelocity(
    simbodyElem->Get<double>("max_transient_velocity"));

  sdf::ElementPtr simbodyContactElem = simbodyElem->GetElement("contact");

  // system wide contact properties, assigned in AddCollisionsToLink()
  this->simbodyPhysicsDPtr->contactMaterialStiffness =
    simbodyContactElem->Get<double>("stiffness");
  this->simbodyPhysicsDPtr->contactMaterialDissipation =
    simbodyContactElem->Get<double>("dissipation");
  this->simbodyPhysicsDPtr->contactMaterialStaticFriction =
    simbodyContactElem->Get<double>("static_friction");
  this->simbodyPhysicsDPtr->contactMaterialDynamicFriction =
    simbodyContactElem->Get<double>("dynamic_friction");
  this->simbodyPhysicsDPtr->contactMaterialViscousFriction =
    simbodyContactElem->Get<double>("viscous_friction");

  // below are not used yet, but should work it into the system
  this->simbodyPhysicsDPtr->contactMaterialPlasticCoefRestitution =
    simbodyContactElem->Get<double>("plastic_coef_restitution");
  this->simbodyPhysicsDPtr->contactMaterialPlasticImpactVelocity =
    simbodyContactElem->Get<double>("plastic_impact_velocity");
  this->simbodyPhysicsDPtr->contactImpactCaptureVelocity =
    simbodyContactElem->Get<double>("override_impact_capture_velocity");
  this->simbodyPhysicsDPtr->contactStictionTransitionVelocity =
    simbodyContactElem->Get<double>("override_stiction_transition_velocity");
}

/////////////////////////////////////////////////
void SimbodyPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::SIMBODY);
    // min_step_size is defined but not yet used
    physicsMsg.set_min_step_size(this->MaxStepSize());
    physicsMsg.set_enable_physics(
        this->simbodyPhysicsDPtr->world->PhysicsEngineEnabled());

    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->simbodyPhysicsDPtr->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
      msgs::Convert(this->world->MagneticField()));
    physicsMsg.set_real_time_update_rate(
        this->simbodyPhysicsDPtr->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(
        this->simbodyPhysicsDPtr->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->simbodyPhysicsDPtr->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->simbodyPhysicsDPtr->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void SimbodyPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_enable_physics())
  {
    this->simbodyPhysicsDPtr->world->SetPhysicsEngineEnabled(
        _msg->enable_physics());
  }

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());

  if (_msg->has_max_step_size())
    this->SetMaxStepSize(_msg->max_step_size());

  /* below will set accuracy for simbody if the messages exist
  // Set integrator accuracy (measured with Richardson Extrapolation)
  if (_msg->has_accuracy())
  {
    this->simbodyPhysicsDPtr->integ->setAccuracy(_msg->simbody().accuracy());
  }

  // Set stiction max slip velocity to make it less stiff.
  if (_msg->has_max_transient_velocity())
  {
    this->simbodyPhysicsDPtr->contact.setTransitionVelocity(
    _msg->simbody().max_transient_velocity());
  }
  */

  /// Make sure all models get at least on update cycle.
  this->simbodyPhysicsDPtr->world->EnableAllModels();
}

//////////////////////////////////////////////////
void SimbodyPhysics::Reset()
{
  this->simbodyPhysicsDPtr->integ->initialize(
      this->simbodyPhysicsDPtr->system.getDefaultState());

  // restore potentially user run-time modified gravity
  this->SetGravity(this->simbodyPhysicsDPtr->world->Gravity());
}

//////////////////////////////////////////////////
void SimbodyPhysics::Init()
{
  this->simbodyPhysicsDPtr->simbodyPhysicsInitialized = true;
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitModel(const physics::ModelPtr _model)
{
  // Before building a new system, transfer all joints in existing
  // models, save Simbody joint states in Gazebo Model.
  const SimTK::State &currentState =
    this->simbodyPhysicsDPtr->integ->getState();
  double stateTime = 0;
  bool simbodyStateSaved = false;

  if (currentState.getSystemStage() != SimTK::Stage::Empty)
  {
    stateTime = currentState.getTime();
    physics::Model_V models = this->simbodyPhysicsDPtr->world->Models();
    for (physics::Model_V::iterator mi = models.begin();
         mi != models.end(); ++mi)
    {
      if ((*mi) != _model)
      {
        physics::Joint_V joints = (*mi)->Joints();
        for (physics::Joint_V::iterator jx = joints.begin();
             jx != joints.end(); ++jx)
        {
          SimbodyJointPtr simbodyJoint =
            std::dynamic_pointer_cast<gazebo::physics::SimbodyJoint>(*jx);
          simbodyJoint->SaveSimbodyState(currentState);
        }

        physics::Link_V links = (*mi)->Links();
        for (physics::Link_V::iterator lx = links.begin();
             lx != links.end(); ++lx)
        {
          SimbodyLinkPtr simbodyLink =
            std::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
          simbodyLink->SaveSimbodyState(currentState);
        }
      }
    }
    simbodyStateSaved = true;
  }

  try
  {
    //------------------------ CREATE SIMBODY SYSTEM ---------------------------
    // Add to Simbody System and populate it with new links and joints
    if (_model->IsStatic())
    {
      SimbodyPhysics::AddStaticModelToSimbodySystem(_model);
    }
    else
    {
      //---------------------- GENERATE MULTIBODY GRAPH ------------------------
      MultibodyGraphMaker mbgraph;
      this->CreateMultibodyGraph(mbgraph, _model);
      // Optional: dump the graph to stdout for debugging or curiosity.
      // mbgraph.dumpGraph(gzdbg);

      SimbodyPhysics::AddDynamicModelToSimbodySystem(mbgraph, _model);
    }
  }
  catch(const std::exception &e)
  {
    gzerr << "Simbody build EXCEPTION: " << e.what() << std::endl;
  }

  try
  {
    //------------------------ CREATE SIMBODY SYSTEM ---------------------------
    // Create a Simbody System and populate it with Subsystems we'll need.
    SimbodyPhysics::InitSimbodySystem();
  }
  catch(const std::exception &e)
  {
    gzerr << "Simbody init EXCEPTION: " << e.what() << std::endl;
  }

  SimTK::State state = this->simbodyPhysicsDPtr->system.realizeTopology();

  // Restore Gazebo saved Joint states
  // back into Simbody state.
  if (simbodyStateSaved)
  {
    // set/retsore state time.
    state.setTime(stateTime);

    physics::Model_V models = this->simbodyPhysicsDPtr->world->Models();
    for (physics::Model_V::iterator mi = models.begin();
         mi != models.end(); ++mi)
    {
      physics::Joint_V joints = (*mi)->Joints();
      for (physics::Joint_V::iterator jx = joints.begin();
           jx != joints.end(); ++jx)
      {
        SimbodyJointPtr simbodyJoint =
          std::dynamic_pointer_cast<gazebo::physics::SimbodyJoint>(*jx);
        simbodyJoint->RestoreSimbodyState(state);
      }
      physics::Link_V links = (*mi)->Links();
      for (physics::Link_V::iterator lx = links.begin();
           lx != links.end(); ++lx)
      {
        SimbodyLinkPtr simbodyLink =
          std::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
        simbodyLink->RestoreSimbodyState(state);
      }
    }
  }

  // initialize integrator from state
  this->simbodyPhysicsDPtr->integ->initialize(state);

  // mark links as initialized
  Link_V links = _model->Links();
  for (Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    physics::SimbodyLinkPtr simbodyLink =
      std::dynamic_pointer_cast<physics::SimbodyLink>(*li);
    if (simbodyLink)
      simbodyLink->SetPhysicsInitialized(true);
    else
      gzerr << "failed to cast link [" << (*li)->Name()
            << "] as simbody link\n";
  }

  // mark joints as initialized
  physics::Joint_V joints = _model->Joints();
  for (physics::Joint_V::iterator ji = joints.begin();
       ji != joints.end(); ++ji)
  {
    SimbodyJointPtr simbodyJoint =
      std::dynamic_pointer_cast<gazebo::physics::SimbodyJoint>(*ji);
    if (simbodyJoint)
      simbodyJoint->SetPhysicsInitialized(true);
    else
      gzerr << "simbodyJoint [" << (*ji)->Name()
            << "]is not a SimbodyJointPtr\n";
  }

  this->simbodyPhysicsDPtr->simbodyPhysicsInitialized = true;
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdateCollision()
{
  std::lock_guard<std::recursive_mutex> lock(
      this->simbodyPhysicsDPtr->physicsUpdateMutex);

  this->simbodyPhysicsDPtr->contactManager->ResetCount();

  // Get all contacts from Simbody
  const SimTK::State &state = this->simbodyPhysicsDPtr->integ->getState();

  // The tracker cannot generate a snapshot without a subsystem
  if (state.getNumSubsystems() == 0)
    return;

  // get contact snapshot
  const SimTK::ContactSnapshot &contactSnapshot =
    this->simbodyPhysicsDPtr->tracker.getActiveContacts(state);

  int numc = contactSnapshot.getNumContacts();

  int count = 0;
  for (int j = 0; j < numc; ++j)
  {
    // get contact stuff from Simbody
    const SimTK::Contact &simbodyContact = contactSnapshot.getContact(j);
    {
      SimTK::ContactSurfaceIndex csi1 = simbodyContact.getSurface1();
      SimTK::ContactSurfaceIndex csi2 = simbodyContact.getSurface2();
      const SimTK::ContactSurface &cs1 =
        this->simbodyPhysicsDPtr->tracker.getContactSurface(csi1);
      const SimTK::ContactSurface &cs2 =
        this->simbodyPhysicsDPtr->tracker.getContactSurface(csi2);

      /// \TODO: See issue #1584
      /// \TODO: below, get collision data from simbody contacts
      Collision *collision1 = nullptr;
      Collision *collision2 = nullptr;
      physics::LinkPtr link1 = nullptr;
      physics::LinkPtr link2 = nullptr;

      /// \TODO: get SimTK::ContactGeometry* from ContactForce somehow
      const SimTK::ContactGeometry &cg1 = cs1.getShape();
      const SimTK::ContactGeometry &cg2 = cs2.getShape();

      /// \TODO: proof of concept only
      /// loop through all link->all collisions and find
      /// this is going to be very very slow, we'll need
      /// something with a void* pointer in simbody
      /// to support something like this.
      physics::Model_V models = this->simbodyPhysicsDPtr->world->Models();
      for (physics::Model_V::iterator mi = models.begin();
           mi != models.end(); ++mi)
      {
        physics::Link_V links = (*mi)->Links();
        for (Link_V::iterator li = links.begin(); li != links.end(); ++li)
        {
          Collision_V collisions = (*li)->Collisions();
          for (Collision_V::iterator ci = collisions.begin();
               ci != collisions.end(); ++ci)
          {
            /// compare SimbodyCollision::CollisionShape() to
            /// ContactGeometry from SimTK::ContactForce
            SimbodyCollisionPtr sc =
              std::dynamic_pointer_cast<physics::SimbodyCollision>(*ci);
            if (sc->CollisionShape() == &cg1)
            {
              collision1 = (*ci).get();
              link1 = (*li);
            }
            else if (sc->CollisionShape() == &cg2)
            {
              collision2 = (*ci).get();
              link2 = (*li);
            }
          }
        }
      }

      // add contacts to the manager. This will return nullptr if no one is
      // listening for contact information.
      Contact *contactFeedback =
        this->simbodyPhysicsDPtr->contactManager->NewContact(collision1,
          collision2, this->simbodyPhysicsDPtr->world->SimTime());

      if (contactFeedback)
      {
        const bool useContactPatch = true;
        if (useContactPatch)
        {
          // get contact patch to get detailed contacts
          // see https://github.com/simbody/simbody/blob/master/examples/
          // ExampleContactPlayground.cpp#L110
          SimTK::ContactPatch patch;
          this->simbodyPhysicsDPtr->system.realize(state,
              SimTK::Stage::Velocity);

          const bool found =
             this->simbodyPhysicsDPtr->contact.calcContactPatchDetailsById(
               state, simbodyContact.getContactId(), patch);

          // loop through details of patch
          if (found)
          {
            for (int i = 0; i < patch.getNumDetails(); ++i)
            {
              if (count >= MAX_CONTACT_JOINTS)
              {
                gzerr << "max contact count [" << MAX_CONTACT_JOINTS
                      << "] exceeded. truncating info.\n";
                continue;
              }
              // gzerr << "count: " << count << "\n";

              // get detail
              const SimTK::ContactDetail &detail = patch.getContactDetail(i);
              // get contact information from simbody and
              // add them to contactFeedback.
              // Store the contact depth
              contactFeedback->depths[count] = detail.getDeformation();

              // Store the contact position
              contactFeedback->positions[count].Set(
                detail.getContactPoint()[0],
                detail.getContactPoint()[1],
                detail.getContactPoint()[2]);

              // Store the contact normal
              contactFeedback->normals[count].Set(
                detail.getContactNormal()[0],
                detail.getContactNormal()[1],
                detail.getContactNormal()[2]);

              // Store the contact forces
              const SimTK::Vec3 f2 = detail.getForceOnSurface2();
              const SimTK::SpatialVec s2 =
                SimTK::SpatialVec(SimTK::Vec3(0, 0, 0), f2);
              /// Get transform from point to CG.
              /// detail.getContactPoint() returns in body frame
              /// per gazebo contact feedback convention.
              const SimTK::Vec3 offset2 = -detail.getContactPoint();
              SimTK::SpatialVec s2cg = SimTK::shiftForceBy(s2, offset2);
              SimTK::Vec3 t2cg = s2cg[0];
              SimTK::Vec3 f2cg = s2cg[1];

              /// shift for body 1
              /// \TODO: generalize wrench shifting below later and add
              /// it to JointWrench class for shifting wrenches around
              /// arbitrarily based on frames.
              ///
              /// shift forces to link1 frame without rotating it first
              ignition::math::Pose3d pose1 = link1->WorldPose();
              ignition::math::Pose3d pose2 = link2->WorldPose();
              const SimTK::Vec3 offset1 = -detail.getContactPoint()
                + SimbodyPhysics::Vector3ToVec3(pose1.Pos() - pose2.Pos());
              SimTK::SpatialVec s1cg = SimTK::shiftForceBy(-s2, offset1);

              /// get torque and force components
              SimTK::Vec3 t1cg = s1cg[0];
              SimTK::Vec3 f1cg = s1cg[1];

              /* actually don't need to do this? confirm that
                 everything is in the world frame!
              /// \TODO: rotate it into link 1 frame, there must be
              /// a clean way to do this in simbody...
              /// my gazebo way of rotating frames for now, to replace with
              /// clean simbody function calls.
              /// rotation from link2 to link1 frame specified in link2 frame
              math::Quaternion rot21 = (pose1 - pose2).rot;
              t1cg = SimbodyPhysics::Vector3ToVec3(
                rot21.RotateVectorReverse(SimbodyPhysics::Vec3ToVector3(t1cg)));
              f1cg = SimbodyPhysics::Vector3ToVec3(
                rot21.RotateVectorReverse(SimbodyPhysics::Vec3ToVector3(f1cg)));

              gzerr << "numc: " << j << "\n";
              gzerr << "count: " << count << "\n";
              gzerr << "index: " << i << "\n";
              gzerr << "offset 2: " << detail.getContactPoint() << "\n";
              gzerr << "s2: " << s2 << "\n";
              gzerr << "s2cg: " << s2cg << "\n";
              gzerr << "f2cg: " << f2cg << "\n";
              gzerr << "t2cg: " << t2cg << "\n";
              gzerr << "offset 1: " << detail.getContactPoint() << "\n";
              gzerr << "s1cg: " << s1cg << "\n";
              gzerr << "f1cg: " << f1cg << "\n";
              gzerr << "t1cg: " << t1cg << "\n";
              */

              // copy.
              contactFeedback->wrench[count].body1Force.Set(
                f1cg[0], f1cg[1], f1cg[2]);
              contactFeedback->wrench[count].body2Force.Set(
                f2cg[0], f2cg[1], f2cg[2]);
              contactFeedback->wrench[count].body1Torque.Set(
                t1cg[0], t1cg[1], t1cg[2]);
              contactFeedback->wrench[count].body2Torque.Set(
                t2cg[0], t2cg[1], t2cg[2]);

              // Increase the counters
              ++count;
              contactFeedback->count = count;
            }
          }
        }
        else  // use single ContactForce
        {
          // // get contact information from simbody ContactForce and
          // // add it to contactFeedback.

          // /// \TODO: confirm the contact depth is zero?
          // contactFeedback->depths[count] = 0.0;

          // // Store the contact position
          // contactFeedback->positions[count].Set(
          //   contactForce.getContactPoint()[0],
          //   contactForce.getContactPoint()[1],
          //   contactForce.getContactPoint()[2]);

          // // Store the contact normal
          // contactFeedback->normals[j].Set(
          //   0, 0, 0);
          //   // contactForce.getContactNormal()[0],
          //   // contactForce.getContactNormal()[1],
          //   // contactForce.getContactNormal()[2]);

          // // Increase the counters
          // ++count;
          // contactFeedback->count = count;
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  std::lock_guard<std::recursive_mutex> lock(
      this->simbodyPhysicsDPtr->physicsUpdateMutex);

  common::Time currTime = this->simbodyPhysicsDPtr->world->RealTime();

  // Simbody cannot step the integrator without a subsystem
  const SimTK::State &s = this->integ->getState();
  if (s.getNumSubsystems() == 0)
    return;

  bool trying = true;
  while (trying &&
      this->simbodyPhysicsDPtr->integ->getTime() <
      this->simbodyPhysicsDPtr->world->SimTime().Double())
  {
    try
    {
      this->simbodyPhysicsDPtr->integ->stepTo(
          this->simbodyPhysicsDPtr->world->SimTime().Double(),
          this->simbodyPhysicsDPtr->world->SimTime().Double());
    }
    catch(const std::exception& e)
    {
      gzerr << "simbody stepTo() failed with message:\n"
            << e.what() << "\nWill stop trying now.\n";
      trying = false;
    }
  }

  this->simbodyPhysicsStepped = true;

  // debug
  // gzerr << "time [" << s.getTime()
  //       << "] q [" << s.getQ()
  //       << "] u [" << s.getU()
  //       << "] dt [" << this->simbodyPhysicsDPtr->stepTimeDouble
  //       << "] t [" << this->simbodyPhysicsDPtr->world->SimTime().Double()
  //       << "]\n";
  // this->lastUpdateTime = currTime;

  // pushing new entity pose into dirtyPoses for visualization
  physics::Model_V models = this->simbodyPhysicsDPtr->world->Models();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = (*mi)->Links();
    for (physics::Link_V::iterator lx = links.begin();
         lx != links.end(); ++lx)
    {
      physics::SimbodyLinkPtr simbodyLink =
        std::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
      ignition::math::Pose3d pose = SimbodyPhysics::Transform2PoseIgn(
        simbodyLink->MobilizedBody().getBodyTransform(s));
      simbodyLink->SetDirtyPose(pose);
      this->simbodyPhysicsDPtr->world->dataPtr->dirtyPoses.push_back(
        std::static_pointer_cast<Entity>(*lx).get());
    }

    physics::Joint_V joints = (*mi)->Joints();
    for (physics::Joint_V::iterator jx = joints.begin();
         jx != joints.end(); ++jx)
    {
      SimbodyJointPtr simbodyJoint =
        std::dynamic_pointer_cast<physics::SimbodyJoint>(*jx);
      simbodyJoint->CacheForceTorque();
    }
  }

  // FIXME:  this needs to happen before forces are applied for the next step
  // FIXME:  but after we've gotten everything from current state
  this->simbodyPhysicsDPtr->discreteForces.clearAllForces(
      this->simbodyPhysicsDPtr->integ->updAdvancedState());
}

//////////////////////////////////////////////////
void SimbodyPhysics::Fini()
{
}

//////////////////////////////////////////////////
LinkPtr SimbodyPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == nullptr)
  {
    gzerr << "Link must have a parent\n";
    return LinkPtr();
  }

  SimbodyLinkPtr link(new SimbodyLink(_parent));
  link->SetWorld(_parent->World());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr SimbodyPhysics::CreateCollision(const std::string &_type,
                                            LinkPtr _parent)
{
  SimbodyCollisionPtr collision(new SimbodyCollision(_parent));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_parent->World());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr SimbodyPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  SimbodyCollisionPtr collision =
    std::dynamic_pointer_cast<SimbodyCollision>(_collision);

  if (_type == "plane")
    shape.reset(new SimbodyPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new SimbodySphereShape(collision));
  else if (_type == "box")
    shape.reset(new SimbodyBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new SimbodyCylinderShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new SimbodyMeshShape(collision));
  else if (_type == "polyline")
    shape.reset(new SimbodyPolylineShape(collision));
  else if (_type == "heightmap")
    shape.reset(new SimbodyHeightmapShape(collision));
  else if (_type == "multiray")
    shape.reset(new SimbodyMultiRayShape(collision));
  else if (_type == "ray")
  {
    if (_collision)
    {
      shape.reset(new SimbodyRayShape(_collision));
    }
    else
    {
      shape.reset(new SimbodyRayShape(
            this->simbodyPhysicsDPtr->world->Physics()));
    }
  }
  else
  {
    gzerr << "Unable to create collision of type[" << _type << "]\n";
  }

  // else if (_type == "map" || _type == "image")
  //   shape.reset(new MapShape(collision));
  return shape;
}

//////////////////////////////////////////////////
JointPtr SimbodyPhysics::CreateJoint(const std::string &_type,
                                     ModelPtr _parent)
{
  JointPtr joint;
  if (_type == "revolute")
  {
    joint.reset(new SimbodyHingeJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "universal")
  {
    joint.reset(new SimbodyUniversalJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "ball")
  {
    joint.reset(new SimbodyBallJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "prismatic")
  {
    joint.reset(new SimbodySliderJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "revolute2")
  {
    joint.reset(new SimbodyHinge2Joint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "screw")
  {
    joint.reset(new SimbodyScrewJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else if (_type == "fixed")
  {
    joint.reset(new SimbodyFixedJoint(
          this->simbodyPhysicsDPtr->dynamicsWorld, _parent));
  }
  else
  {
    gzerr << "Unable to create joint of type[" << _type << "]";
  }

  return joint;
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->simbodyPhysicsDPtr->world->SetGravitySDF(_gravity);

  {
    std::lock_guard<std::recursive_mutex> lock(
        this->simbodyPhysicsDPtr->physicsUpdateMutex);
    if (this->simbodyPhysicsDPtr->simbodyPhysicsInitialized &&
        this->simbodyPhysicsDPtr->world->ModelCount() > 0)
    {
      this->simbodyPhysicsDPtr->gravity.setGravityVector(
          this->simbodyPhysicsDPtr->integ->updAdvancedState(),
          SimbodyPhysics::Vector3ToVec3(_gravity));
    }
    else
    {
      this->simbodyPhysicsDPtr->gravity.setDefaultGravityVector(
        SimbodyPhysics::Vector3ToVec3(_gravity));
    }
  }
}

//////////////////////////////////////////////////
void SimbodyPhysics::DebugPrint() const
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::CreateMultibodyGraph(
  SimTK::MultibodyGraphMaker &_mbgraph, const physics::ModelPtr _model)
{
  // Step 1: Tell MultibodyGraphMaker about joints it should know about.
  // Note: "weld" and "free" are always predefined at 0 and 6 dofs, resp.
  //                  Gazebo name  #dofs     Simbody equivalent
  _mbgraph.addJointType(TypeString(physics::Base::HINGE_JOINT),  1);
  _mbgraph.addJointType(TypeString(physics::Base::HINGE2_JOINT), 2);
  _mbgraph.addJointType(TypeString(physics::Base::SLIDER_JOINT), 1);
  _mbgraph.addJointType(TypeString(physics::Base::UNIVERSAL_JOINT), 2);
  _mbgraph.addJointType(TypeString(physics::Base::SCREW_JOINT), 1);
  _mbgraph.addJointType(TypeString(physics::Base::FIXED_JOINT), 0);

  // Simbody has a Ball constraint that is a good choice if you need to
  // break a loop at a ball joint.
  // _mbgraph.addJointType(TypeString(physics::Base::BALL_JOINT), 3, true);
  // skip loop joints for now
  _mbgraph.addJointType(TypeString(physics::Base::BALL_JOINT), 3, false);

  // Step 2: Tell it about all the links we read from the input file,
  // starting with world, and provide a reference pointer.
  _mbgraph.addBody("world", SimTK::Infinity,
                  false);

  physics::Link_V links = _model->Links();
  for (physics::Link_V::iterator li = links.begin();
       li != links.end(); ++li)
  {
    SimbodyLinkPtr simbodyLink = std::dynamic_pointer_cast<SimbodyLink>(*li);

    // gzerr << "debug : " << (*li)->Name() << "\n";

    if (simbodyLink)
      _mbgraph.addBody((*li)->Name(), (*li)->Inertia().Mass(),
                      simbodyLink->MustBeBaseLink(), (*li).get());
    else
      gzerr << "simbodyLink [" << (*li)->Name()
            << "]is not a SimbodyLinkPtr\n";
  }

  // Step 3: Tell it about all the joints we read from the input file,
  // and provide a reference pointer.
  physics::Joint_V joints = _model->Joints();
  for (physics::Joint_V::iterator ji = joints.begin();
       ji != joints.end(); ++ji)
  {
    SimbodyJointPtr simbodyJoint =
      std::dynamic_pointer_cast<SimbodyJoint>(*ji);
    if (simbodyJoint)
    {
      if ((*ji)->Parent() && (*ji)->Child())
      {
        _mbgraph.addJoint((*ji)->Name(), TypeString((*ji)->Type()),
            (*ji)->Parent()->Name(), (*ji)->Child()->Name(),
            simbodyJoint->MustBreakLoopHere(), (*ji).get());
      }
      else if ((*ji)->Child())
      {
        _mbgraph.addJoint((*ji)->Name(), TypeString((*ji)->Type()),
            "world", (*ji)->Child()->Name(),
            simbodyJoint->MustBreakLoopHere(), (*ji).get());
      }
      else
      {
        gzerr << "simbodyJoint [" << (*ji)->Name()
              << "] does not have a valid child link, which is required\n";
      }
    }
    else
    {
      gzerr << "simbodyJoint [" << (*ji)->Name()
            << "]is not a SimbodyJointPtr\n";
    }
  }

  // Setp 4. Generate the multibody graph.
  _mbgraph.generateGraph();
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitSimbodySystem()
{
  // Set stiction max slip velocity to make it less stiff.
  // this->simbodyPhysicsDPtr->contact.setTransitionVelocity(0.01);
  // now done in Load using sdf

  // Specify gravity (read in above from world).
  if (!math::equal(this->simbodyPhysicsDPtr->world->Gravity().Length(), 0.0))
  {
    this->simbodyPhysicsDPtr->gravity.setDefaultGravityVector(
      SimbodyPhysics::Vector3ToVec3(
        this->simbodyPhysicsDPtr->world->Gravity()));
  }
  else
  {
    this->simbodyPhysicsDPtr->gravity.setDefaultMagnitude(0.0);
  }
}

//////////////////////////////////////////////////
void SimbodyPhysics::AddStaticModelToSimbodySystem(
    const physics::ModelPtr _model)
{
  physics::Link_V links = _model->Links();
  for (physics::Link_V::iterator li = links.begin();
       li != links.end(); ++li)
  {
    SimbodyLinkPtr simbodyLink = std::dynamic_pointer_cast<SimbodyLink>(*li);
    if (simbodyLink)
    {
      this->AddCollisionsToLink(simbodyLink.get(),
          this->simbodyPhysicsDPtr->matter.updGround(), ContactCliqueId());
      simbodyLink->MobilizedBody() =
        this->simbodyPhysicsDPtr->matter.updGround();
    }
    else
      gzerr << "simbodyLink [" << (*li)->Name()
            << "]is not a SimbodyLinkPtr\n";
  }
}

//////////////////////////////////////////////////
void SimbodyPhysics::AddDynamicModelToSimbodySystem(
  const SimTK::MultibodyGraphMaker &_mbgraph,
  const physics::ModelPtr /*_model*/)
{
  // Generate a contact clique we can put collision geometry in to prevent
  // self-collisions.
  // \TODO: put this in a gazebo::physics::SimbodyModel class
  ContactCliqueId modelClique = ContactSurface::createNewContactClique();

  // Will specify explicitly when needed
  // Record the MobilizedBody for the World link.
  // model.links.updLink(0).masterMobod =
  // this->simbodyPhysicsDPtr->matter.Ground();

  // Run through all the mobilizers in the multibody graph, adding a Simbody
  // MobilizedBody for each one. Also add visual and collision geometry to the
  // bodies when they are mobilized.
  for (int mobNum = 0; mobNum < _mbgraph.getNumMobilizers(); ++mobNum)
  {
    // Get a mobilizer from the graph, then extract its corresponding
    // joint and bodies. Note that these don't necessarily have equivalents
    // in the GazeboLink and GazeboJoint inputs.
    const MultibodyGraphMaker::Mobilizer& mob = _mbgraph.getMobilizer(mobNum);
    const std::string& type = mob.getJointTypeName();

    // The inboard body always corresponds to one of the input links,
    // because a slave link is always the outboard body of a mobilizer.
    // The outboard body may be slave, but its master body is one of the
    // Gazebo input links.
    const bool isSlave = mob.isSlaveMobilizer();
    // note: do not use boost shared pointer here, on scope out the
    // original pointer get scrambled
    SimbodyLink* gzInb = static_cast<SimbodyLink*>(mob.getInboardBodyRef());
    SimbodyLink* gzOutb =
      static_cast<SimbodyLink*>(mob.getOutboardMasterBodyRef());

    const MassProperties massProps =
        gzOutb->EffectiveMassProps(mob.getNumFragments());

    // debug
    // if (gzInb)
    //   gzerr << "debug: Inb: " << gzInb->Name() << "\n";
    // if (gzOutb)
    //   gzerr << "debug: Outb: " << gzOutb->Name()
    //         << " mass: " << gzOutb->Inertial()->Mass()
    //         << " efm: " << massProps
    //         << "\n";

    // This will reference the new mobilized body once we create it.
    MobilizedBody mobod;

    MobilizedBody parentMobod =
      gzInb == nullptr ? this->simbodyPhysicsDPtr->matter.Ground() :
      gzInb->MobilizedBody();

    if (mob.isAddedBaseMobilizer())
    {
      // There is no corresponding Gazebo joint for this mobilizer.
      // Create the joint and set its default position to be the default
      // pose of the base link relative to the Ground frame.
      // Currently only `free` is allowed, we may add more types later
      GZ_ASSERT(type == "free", "type is not 'free', not allowed.");
      if (type == "free")
      {
        MobilizedBody::Free freeJoint(
            parentMobod,  Transform(),
            massProps,    Transform());

        SimTK::Transform inboardXML;
        if (gzInb == nullptr)
        {
          // GZ_ASSERT(gzOutb, "must be here");
          physics::Model *model = gzOutb->ParentModel();
          inboardXML =
            ~SimbodyPhysics::Pose2Transform(model->WorldPose());
        }
        else
        {
          inboardXML =
            SimbodyPhysics::Pose2Transform(gzInb->RelativePose());
        }

        SimTK::Transform outboardXML =
          SimbodyPhysics::Pose2Transform(gzOutb->RelativePose());

        // defX_ML link frame specified in model frame
        freeJoint.setDefaultTransform(~inboardXML * outboardXML);
        mobod = freeJoint;
      }
    }
    else
    {
      // This mobilizer does correspond to one of the input joints.
      // note: do not use boost shared pointer here, on scope out the
      // original pointer get scrambled
      SimbodyJoint *gzJoint = static_cast<SimbodyJoint*>(mob.getJointRef());
      const bool isReversed = mob.isReversedFromJoint();

      // Find inboard and outboard frames for the mobilizer; these are
      // parent and child frames or the reverse.

      const Transform &X_IF0 = isReversed ? gzJoint->XCB() : gzJoint->XPA();
      const Transform &X_OM0 = isReversed ? gzJoint->XPA() : gzJoint->XCB();

      const MobilizedBody::Direction direction =
          isReversed ? MobilizedBody::Reverse : MobilizedBody::Forward;

      if (type == "free")
      {
        MobilizedBody::Free freeJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        Transform defX_FM = isReversed ? Transform(~gzJoint->DefxAB())
                                       : gzJoint->DefxAB();
        freeJoint.setDefaultTransform(defX_FM);
        mobod = freeJoint;
      }
      else if (type == "screw")
      {
        UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(gzJoint->LocalAxis(0))));

        double pitch =
          dynamic_cast<physics::SimbodyScrewJoint*>(gzJoint)->ThreadPitch(0);

        if (math::equal(pitch, 0.0))
        {
          gzerr << "thread pitch should not be zero (joint is a slider?)"
                << " using pitch = 1.0e6\n";
          pitch = 1.0e6;
        }

        // Simbody's screw joint axis (both rotation and translation) is along Z
        Rotation R_JZ(axis, ZAxis);
        Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
        MobilizedBody::Screw screwJoint(
            parentMobod,      X_IF,
            massProps,        X_OM,
            -1.0/pitch,
            direction);
        mobod = screwJoint;

        gzdbg << "Setting limitForce[0] for [" << gzJoint->Name() << "]\n";

        double low = gzJoint->LowerLimit(0u).Radian();
        double high = gzJoint->UpperLimit(0u).Radian();

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->SetLimitForce(0,
          Force::MobilityLinearStop(this->simbodyPhysicsDPtr->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->StopStiffness(0),
          gzJoint->StopDissipation(0), low, high));

        // gzdbg << "SimbodyPhysics SetDamping ("
        //       << gzJoint->DampingCoefficient()
        //       << ")\n";
        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->SetDamper(0,
          Force::MobilityLinearDamper(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->Damping(0)));

        // add spring (stiffness proportional to mass)
        gzJoint->SetSpring(0,
          Force::MobilityLinearSpring(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->SpringStiffness(0),
              gzJoint->SpringReferencePosition(0)));
      }
      else if (type == "universal")
      {
        UnitVec3 axis1(SimbodyPhysics::Vector3ToVec3(
          gzJoint->AxisFrameOffset(0).RotateVector(
          gzJoint->LocalAxis(UniversalJoint<Joint>::AXIS_PARENT))));
        /// \TODO: check if this is right, or GetAxisFrameOffset(1) is needed.
        UnitVec3 axis2(SimbodyPhysics::Vector3ToVec3(
          gzJoint->AxisFrameOffset(0).RotateVector(
          gzJoint->LocalAxis(UniversalJoint<Joint>::AXIS_CHILD))));

        // Simbody's univeral joint is along axis1=Y and axis2=X
        // note X and Y are reversed because Simbody defines universal joint
        // rotation in body-fixed frames, whereas Gazebo/ODE uses space-fixed
        // frames.
        Rotation R_JF(axis1, XAxis, axis2, YAxis);
        Transform X_IF(X_IF0.R()*R_JF, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JF, X_OM0.p());
        MobilizedBody::Universal uJoint(
            parentMobod,      X_IF,
            massProps,        X_OM,
            direction);
        mobod = uJoint;

        for (unsigned int nj = 0; nj < 2; ++nj)
        {
          double low = gzJoint->LowerLimit(nj).Radian();
          double high = gzJoint->UpperLimit(nj).Radian();

          // initialize stop stiffness and dissipation from joint parameters
          gzJoint->SetLimitForce(nj,
            Force::MobilityLinearStop(this->simbodyPhysicsDPtr->forces, mobod,
            SimTK::MobilizerQIndex(nj), gzJoint->StopStiffness(nj),
            gzJoint->StopDissipation(nj), low, high));

          // gzdbg << "stop stiffness [" << gzJoint->StopStiffness(nj)
          //       << "] low [" << low
          //       << "] high [" << high
          //       << "]\n";

          // gzdbg << "SimbodyPhysics SetDamping ("
          //       << gzJoint->DampingCoefficient()
          //       << ")\n";
          // Create a damper for every joint even if damping coefficient
          // is zero.  This will allow user to change damping coefficients
          // on the fly.
          gzJoint->SetDamper(nj,
            Force::MobilityLinearDamper(
                this->simbodyPhysicsDPtr->forces, mobod, nj,
                gzJoint->Damping(nj)));

          // add spring (stiffness proportional to mass)
          gzJoint->SetSpring(nj,
            Force::MobilityLinearSpring(
                this->simbodyPhysicsDPtr->forces, mobod, nj,
                gzJoint->SpringStiffness(nj),
                gzJoint->SpringReferencePosition(nj)));
        }
      }
      else if (type == "revolute")
      {
        // rotation from axis frame to child link frame
        // simbody assumes links are in child link frame, but gazebo
        // sdf 1.4 and earlier assumes joint axis are defined in model frame.
        // Use function Joint::AxisFrame() to remedy this situation.
        // Joint::AxisFrame() returns the frame joint axis is defined:
        // either model frame or child link frame.
        // simbody always assumes axis is specified in the child link frame.
        // \TODO: come up with a test case where we might need to
        // flip transform based on isReversed flag.
        UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(
            gzJoint->LocalAxis(0))));

        // gzerr << "[" << gzJoint->AxisFrameOffset(0).GetAsEuler()
        //       << "] ["
        //       << gzJoint->AxisFrameOffset(0).RotateVector(
        //          gzJoint->LocalAxis(0)) << "]\n";

        // Simbody's pin is along Z
        Rotation R_JZ(axis, ZAxis);
        Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
        MobilizedBody::Pin pinJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = pinJoint;

        double low = gzJoint->LowerLimit(0u).Radian();
        double high = gzJoint->UpperLimit(0u).Radian();

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->SetLimitForce(0,
          Force::MobilityLinearStop(this->simbodyPhysicsDPtr->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->StopStiffness(0),
          gzJoint->StopDissipation(0), low, high));

        // gzdbg << "SimbodyPhysics SetDamping ("
        //       << gzJoint->DampingCoefficient()
        //       << ")\n";
        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->SetDamper(0,
          Force::MobilityLinearDamper(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->Damping(0)));

        // add spring (stiffness proportional to mass)
        gzJoint->SetSpring(0,
          Force::MobilityLinearSpring(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->SpringStiffness(0),
              gzJoint->SpringReferencePosition(0)));
      }
      else if (type == "prismatic")
      {
        UnitVec3 axis(SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(
            gzJoint->LocalAxis(0))));

        // Simbody's slider is along X
        Rotation R_JX(axis, XAxis);
        Transform X_IF(X_IF0.R()*R_JX, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JX, X_OM0.p());
        MobilizedBody::Slider sliderJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = sliderJoint;

        double low = gzJoint->LowerLimit(0u).Radian();
        double high = gzJoint->UpperLimit(0u).Radian();

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->SetLimitForce(0,
          Force::MobilityLinearStop(this->simbodyPhysicsDPtr->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->StopStiffness(0),
          gzJoint->StopDissipation(0), low, high));

        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->SetDamper(0,
          Force::MobilityLinearDamper(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->Damping(0)));

        // add spring (stiffness proportional to mass)
        gzJoint->SetSpring(0,
          Force::MobilityLinearSpring(
              this->simbodyPhysicsDPtr->forces, mobod, 0,
              gzJoint->SpringStiffness(0),
              gzJoint->SpringReferencePosition(0)));
      }
      else if (type == "ball")
      {
        MobilizedBody::Ball ballJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        Rotation defR_FM = isReversed
            ? Rotation(~gzJoint->DefxAB().R())
            : gzJoint->DefxAB().R();
        ballJoint.setDefaultRotation(defR_FM);
        mobod = ballJoint;
      }
      else if (type == "fixed")
      {
        MobilizedBody::Weld fixedJoint(
            parentMobod,  X_IF0,
            massProps,    X_OM0);
        mobod = fixedJoint;
      }
      else
      {
        gzerr << "Simbody joint type [" << type << "] not implemented.\n";
      }

      // Created a mobilizer that corresponds to gzJoint. Keep track.
      gzJoint->SetMobod(mobod);
      gzJoint->SetIsReversed(isReversed);
    }

    // Link gzOutb has been mobilized; keep track for later.
    if (isSlave)
      gzOutb->AddSlaveMobod(mobod);
    else
      gzOutb->MobilizedBody() = mobod;

    // A mobilizer has been created; now add the collision
    // geometry for the new mobilized body.
    this->AddCollisionsToLink(gzOutb, mobod, modelClique);
  }

  // Weld the slaves to their masters.
  physics::Model_V models = this->simbodyPhysicsDPtr->world->Models();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = (*mi)->Links();
    for (physics::Link_V::iterator lx = links.begin();
         lx != links.end(); ++lx)
    {
      physics::SimbodyLinkPtr link =
        std::dynamic_pointer_cast<physics::SimbodyLink>(*lx);

      if (link->SlaveMobodsCount() == 0)
        continue;

      for (unsigned i = 0; i < link->SlaveMobodsCount(); ++i)
      {
        Constraint::Weld weld(link->MobilizedBody(), link->SlaveMobod(i));

        // in case we want to know later
        link->AddSlaveWeld(weld);
      }
    }
  }

  //   leave out optimization
  // // Add the loop joints if any.
  // for (int lcx=0; lcx < _mbgraph.getNumLoopConstraints(); ++lcx) {
  //     const MultibodyGraphMaker::LoopConstraint& loop =
  //         _mbgraph.getLoopConstraint(lcx);

  //     SimbodyJointPtr joint(loop.getJointRef());
  //     SimbodyLinkPtr  parent(loop.getParentBodyRef());
  //     SimbodyLinkPtr  child(loop.getChildBodyRef());

  //     if (joint.type == "weld") {
  //         Constraint::Weld weld(parent.masterMobod, joint.XPA(),
  //                               child.masterMobod,  joint.XCB());
  //         joint.constraint = weld;
  //     } else if (joint.type == "ball") {
  //         Constraint::Ball ball(parent.masterMobod, joint.XPA().p(),
  //                               child.masterMobod,  joint.XCB().p());
  //         joint.constraint = ball;
  //     } else if (joint.type == "free") {
  //         // A "free" loop constraint is no constraint at all so we can
  //         // just ignore it. It might be more convenient if there were
  //         // a 0-constraint Constraint::Free, just as there is a 0-mobility
  //         // MobilizedBody::Weld.
  //     } else
  //         throw std::runtime_error(
  //             "Unrecognized loop constraint type '" + joint.type + "'.");
  // }
}

std::string SimbodyPhysics::TypeString(physics::Base::EntityType _type)
{
  // switch (_type)
  // {
  //   case physics::Base::BALL_JOINT:
  //     gzerr << "here\n";
  //     return "ball";
  //     break;
  //   case physics::Base::HINGE2_JOINT:
  //     return "revolute2";
  //     break;
  //   case physics::Base::HINGE_JOINT:
  //     return "revolute";
  //     break;
  //   case physics::Base::SLIDER_JOINT:
  //     return "prismatic";
  //     break;
  //   case physics::Base::SCREW_JOINT:
  //     return "screw";
  //     break;
  //   case physics::Base::UNIVERSAL_JOINT:
  //     return "universal";
  //     break;
  //   default:
  //     gzerr << "Unrecognized joint type\n";
  //     return "UNRECOGNIZED";
  // }

  if (_type & physics::Base::BALL_JOINT)
    return "ball";
  else if (_type & physics::Base::HINGE2_JOINT)
      return "revolute2";
  else if (_type & physics::Base::HINGE_JOINT)
      return "revolute";
  else if (_type & physics::Base::SLIDER_JOINT)
      return "prismatic";
  else if (_type & physics::Base::SCREW_JOINT)
      return "screw";
  else if (_type & physics::Base::UNIVERSAL_JOINT)
      return "universal";
  else if (_type & physics::Base::FIXED_JOINT)
      return "fixed";

  gzerr << "Unrecognized joint type\n";
  return "UNRECOGNIZED";
}

/////////////////////////////////////////////////
void SimbodyPhysics::SetSeed(const uint32_t /*_seed*/)
{
  gzerr << "SimbodyPhysics::SetSeed not implemented\n";
}

/////////////////////////////////////////////////
void SimbodyPhysics::AddCollisionsToLink(const physics::SimbodyLink *_link,
  MobilizedBody &_mobod, ContactCliqueId _modelClique)
{
  // TODO: Edit physics::Surface class to support these properties
  // Define a material to use for contact. This is not very stiff.
  // use stiffness of 1e8 and dissipation of 1000.0 to approximate inelastic
  // collision. but 1e6 and 10 seems sufficient when TransitionVelocity is
  // reduced from 0.1 to 0.01
  SimTK::ContactMaterial material(
      this->simbodyPhysicsDPtr->contactMaterialStiffness,
      this->simbodyPhysicsDPtr->contactMaterialDissipation,
      this->simbodyPhysicsDPtr->contactMaterialStaticFriction,
      this->simbodyPhysicsDPtr->contactMaterialDynamicFriction,
      this->simbodyPhysicsDPtr->contactMaterialViscousFriction);

  // Debug: works for SpawnDrop
  // SimTK::ContactMaterial material(1e6,   // stiffness
  //                                 10.0,  // dissipation
  //                                 0.7,   // mu_static
  //                                 0.5,   // mu_dynamic
  //                                 0.5);  // mu_viscous

  bool addModelClique = _modelClique.isValid() && !_link->SelfCollide();

  // COLLISION
  Collision_V collisions =  _link->Collisions();
  for (Collision_V::iterator ci = collisions.begin();
                             ci != collisions.end(); ++ci)
  {
    Transform X_LC =
      SimbodyPhysics::Pose2Transform((*ci)->RelativePose());

    // use pointer to store CollisionGeometry
    SimbodyCollisionPtr sc =
      std::dynamic_pointer_cast<physics::SimbodyCollision>(*ci);

    switch ((*ci)->ShapeType() & (~physics::Entity::SHAPE))
    {
      case physics::Entity::PLANE_SHAPE:
      {
        std::shared_ptr<physics::PlaneShape> p =
          std::dynamic_pointer_cast<physics::PlaneShape>((*ci)->Shape());

        // by default, simbody HalfSpace normal is in the -X direction
        // rotate it based on normal vector specified by user
        // Create a rotation whos x-axis is in the
        // negative normal vector direction
        Vec3 normal = SimbodyPhysics::Vector3ToVec3(p->Normal());
        Rotation R_XN(-UnitVec3(normal), XAxis);

        ContactSurface surface(ContactGeometry::HalfSpace(), material);

        if (addModelClique)
            surface.joinClique(_modelClique);

        int surfNum = _mobod.updBody().addContactSurface(R_XN, surface);

        // store ContactGeometry pointer in SimbodyCollision object
        SimTK::ContactSurface &contactSurf =
          _mobod.updBody().updContactSurface(surfNum);
        sc->SetCollisionShape(&contactSurf.updShape());
      }
      break;

      case physics::Entity::SPHERE_SHAPE:
      {
        std::shared_ptr<physics::SphereShape> s =
          std::dynamic_pointer_cast<physics::SphereShape>((*ci)->Shape());
        double r = s->Radius();
        ContactSurface surface(ContactGeometry::Sphere(r), material);
        if (addModelClique)
            surface.joinClique(_modelClique);
        int surfNum = _mobod.updBody().addContactSurface(X_LC, surface);

        // store ContactGeometry pointer in SimbodyCollision object
        SimTK::ContactSurface &contactSurf =
          _mobod.updBody().updContactSurface(surfNum);
        sc->SetCollisionShape(&contactSurf.updShape());
      }
      break;

      case physics::Entity::CYLINDER_SHAPE:
      {
        std::shared_ptr<physics::CylinderShape> c =
          std::dynamic_pointer_cast<physics::CylinderShape>((*ci)->Shape());
        double r = c->Radius();
        double len = c->Length();

        // chunky hexagonal shape
        const int resolution = 1;
        const PolygonalMesh mesh = PolygonalMesh::
            createCylinderMesh(ZAxis, r, len/2, resolution);
        const ContactGeometry::TriangleMesh triMesh(mesh);
        ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // Vec3 esz = Vec3(r, r, len/2);  // Use ellipsoid instead
        // ContactSurface surface(ContactGeometry::Ellipsoid(esz),
        //                        material);

        if (addModelClique)
            surface.joinClique(_modelClique);
        int surfNum = _mobod.updBody().addContactSurface(X_LC, surface);

        // store ContactGeometry pointer in SimbodyCollision object
        SimTK::ContactSurface &contactSurf =
          _mobod.updBody().updContactSurface(surfNum);
        sc->SetCollisionShape(&contactSurf.updShape());
      }
      break;

      case physics::Entity::BOX_SHAPE:
      {
        Vec3 hsz = SimbodyPhysics::Vector3ToVec3(
          (std::dynamic_pointer_cast<physics::BoxShape>(
          (*ci)->Shape()))->Size())/2;

        /// \TODO: harcoded resolution, make collision resolution
        /// an adjustable parameter (#980)
        // number times to chop the longest side.
        const int resolution = 6;
        // const int resolution = 10 * (int)(max(hsz)/min(hsz) + 0.5);
        const PolygonalMesh mesh = PolygonalMesh::
            createBrickMesh(hsz, resolution);
        const ContactGeometry::TriangleMesh triMesh(mesh);
        ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // ContactSurface surface(ContactGeometry::Ellipsoid(hsz),
        //                        material);

        if (addModelClique)
            surface.joinClique(_modelClique);
        int surfNum = _mobod.updBody().addContactSurface(X_LC, surface);

        // store ContactGeometry pointer in SimbodyCollision object
        SimTK::ContactSurface &contactSurf =
          _mobod.updBody().updContactSurface(surfNum);
        sc->SetCollisionShape(&contactSurf.updShape());
      }
      break;
      default:
        gzerr << "Collision type [" << (*ci)->ShapeType()
              << "] unimplemented\n";
        break;
    }
  }
}

/////////////////////////////////////////////////
std::string SimbodyPhysics::Type() const
{
  return "simbody";
}

/////////////////////////////////////////////////
SimTK::MultibodySystem *SimbodyPhysics::DynamicsWorld() const
{
  return this->simbodyPhysicsDPtr->dynamicsWorld;
}

/////////////////////////////////////////////////
SimTK::Quaternion SimbodyPhysics::QuadToQuad(const math::Quaternion &_q)
{
  return SimTK::Quaternion(_q.w, _q.x, _q.y, _q.z);
}

/////////////////////////////////////////////////
SimTK::Quaternion SimbodyPhysics::QuadToQuad(
    const ignition::math::Quaterniond &_q)
{
  return SimTK::Quaternion(_q.W(), _q.X(), _q.Y(), _q.Z());
}

/////////////////////////////////////////////////
math::Quaternion SimbodyPhysics::QuadToQuad(const SimTK::Quaternion &_q)
{
  return math::Quaternion(_q[0], _q[1], _q[2], _q[3]);
}

/////////////////////////////////////////////////
SimTK::Vec3 SimbodyPhysics::Vector3ToVec3(const math::Vector3 &_v)
{
  return SimTK::Vec3(_v.x, _v.y, _v.z);
}

/////////////////////////////////////////////////
SimTK::Vec3 SimbodyPhysics::Vector3ToVec3(const ignition::math::Vector3d &_v)
{
  return SimTK::Vec3(_v.X(), _v.Y(), _v.Z());
}

/////////////////////////////////////////////////
math::Vector3 SimbodyPhysics::Vec3ToVector3(const SimTK::Vec3 &_v)
{
  return math::Vector3(_v[0], _v[1], _v[2]);
}

/////////////////////////////////////////////////
ignition::math::Vector3d SimbodyPhysics::Vec3ToVector3Ign(const SimTK::Vec3 &_v)
{
  return ignition::math::Vector3d(_v[0], _v[1], _v[2]);
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyPhysics::Pose2Transform(
    const ignition::math::Pose3d &_pose)
{
  SimTK::Quaternion q(_pose.Rot().W(), _pose.Rot().X(), _pose.Rot().Y(),
                   _pose.Rot().Z());
  SimTK::Vec3 v(_pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z());
  SimTK::Transform frame(SimTK::Rotation(q), v);
  return frame;
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyPhysics::Pose2Transform(const math::Pose &_pose)
{
  SimTK::Quaternion q(_pose.rot.w, _pose.rot.x, _pose.rot.y,
                   _pose.rot.z);
  SimTK::Vec3 v(_pose.pos.x, _pose.pos.y, _pose.pos.z);
  SimTK::Transform frame(SimTK::Rotation(q), v);
  return frame;
}

/////////////////////////////////////////////////
ignition::math::Pose3d SimbodyPhysics::Transform2PoseIgn(
    const SimTK::Transform &_xAB)
{
  SimTK::Quaternion q(_xAB.R());
  const SimTK::Vec4 &qv = q.asVec4();
  return ignition::math::Pose3d(
      ignition::math::Vector3d(_xAB.p()[0], _xAB.p()[1], _xAB.p()[2]),
    ignition::math::Quaterniond(qv[0], qv[1], qv[2], qv[3]));
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyPhysics::Pose(sdf::ElementPtr _element)
{
  const ignition::math::Pose3d pose = _element->Get<
    ignition::math::Pose3d>("pose");
  return Pose2Transform(pose);
}

/////////////////////////////////////////////////
std::string SimbodyPhysics::TypeString(const unsigned int _type)
{
  return TypeString(physics::Base::EntityType(_type));
}

//////////////////////////////////////////////////
boost::any SimbodyPhysics::Param(const std::string &_key) const
{
  boost::any value;
  this->Param(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool SimbodyPhysics::Param(const std::string &_key, boost::any &_value) const
{
  if (_key == "solver_type")
  {
    _value = std::string("Spatial Algebra and Elastic Foundation");
  }
  else if (_key == "integrator_type")
  {
    _value = this->simbodyPhysicsDPtr->integratorType;
  }
  else if (_key == "accuracy")
  {
    if (this->simbodyPhysicsDPtr->integ)
      _value = this->simbodyPhysicsDPtr->integ->getAccuracyInUse();
    else
      _value = 0.0f;
  }
  else if (_key == "max_transient_velocity")
  {
    _value = this->simbodyPhysicsDPtr->contact.getTransitionVelocity();
  }
  else
  {
    return PhysicsEngine::Param(_key, _value);
  }
  return true;
}

//////////////////////////////////////////////////
bool SimbodyPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  try
  {
    if (_key == "accuracy")
    {
      this->simbodyPhysicsDPtr->integ->setAccuracy(
          boost::any_cast<double>(_value));
    }
    else if (_key == "max_transient_velocity")
    {
      this->simbodyPhysicsDPtr->contact.setTransitionVelocity(
          boost::any_cast<double>(_value));
    }
    else if (_key == "stiffness")
    {
      this->simbodyPhysicsDPtr->contactMaterialStiffness =
        boost::any_cast<double>(_value);
    }
    else if (_key == "dissipation")
    {
      this->simbodyPhysicsDPtr->contactMaterialDissipation =
        boost::any_cast<double>(_value);
    }
    else if (_key == "plastic_coef_restitution")
    {
      this->simbodyPhysicsDPtr->contactMaterialPlasticCoefRestitution =
          boost::any_cast<double>(_value);
    }
    else if (_key == "plastic_impact_velocity")
    {
      this->simbodyPhysicsDPtr->contactMaterialPlasticImpactVelocity =
          boost::any_cast<double>(_value);
    }
    else if (_key == "static_friction")
    {
      this->simbodyPhysicsDPtr->contactMaterialStaticFriction =
        boost::any_cast<double>(_value);
    }
    else if (_key == "dynamic_friction")
    {
      this->simbodyPhysicsDPtr->contactMaterialDynamicFriction =
        boost::any_cast<double>(_value);
    }
    else if (_key == "viscous_friction")
    {
      this->simbodyPhysicsDPtr->contactMaterialViscousFriction =
        boost::any_cast<double>(_value);
    }
    else if (_key == "override_impact_capture_velocity")
    {
      this->simbodyPhysicsDPtr->contactMaterialPlasticImpactVelocity =
          boost::any_cast<double>(_value);
    }
    else if (_key == "override_stiction_transition_velocity")
    {
      this->simbodyPhysicsDPtr->contactImpactCaptureVelocity =
        boost::any_cast<double>(_value);
    }
    else
    {
      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "SimbodyPhysics::SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool SimbodyPhysics::PhysicsInitialized() const
{
  return this->simbodyPhysicsDPtr->simbodyPhysicsInitialized;
}

/////////////////////////////////////////////////
SimTK::Force::DiscreteForces &SimbodyPhysics::DiscreteForces() const
{
  return this->simbodyPhysicsDPtr->discreteForces;
}

/////////////////////////////////////////////////
SimTK::MultibodySystem &SimbodyPhysics::System() const
{
  return this->simbodyPhysicsDPtr->system;
}

/////////////////////////////////////////////////
SimTK::Integrator *SimbodyPhysics::Integ() const
{
  return this->simbodyPhysicsDPtr->integ;
}

/////////////////////////////////////////////////
bool SimbodyPhysics::PhysicsStepped() const
{
  return this->simbodyPhysicsDPtr->simbodyPhysicsStepped;
}

/////////////////////////////////////////////////
SimTK::Force::Gravity &SimbodyPhysics::SimbodyGravity() const
{
  return this->simbodyPhysicsDPtr->gravity;
}
