/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <ignition/common/Profiler.hh>

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
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/simbody/SimbodyPhysics.hh"

typedef boost::shared_ptr<gazebo::physics::SimbodyJoint> SimbodyJointPtr;

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("simbody", SimbodyPhysics)

//////////////////////////////////////////////////
SimbodyPhysics::SimbodyPhysics(WorldPtr _world)
    : PhysicsEngine(_world), system(), matter(system), forces(system),
      gravity(forces, matter, -SimTK::ZAxis, 0),
      discreteForces(forces, matter),
      tracker(system), contact(system, tracker),  integ(nullptr)
      , contactMaterialStiffness(0.0)
      , contactMaterialDissipation(0.0)
      , contactMaterialPlasticCoefRestitution(0.0)
      , contactMaterialPlasticImpactVelocity(0.0)
      , contactMaterialStaticFriction(0.0)
      , contactMaterialDynamicFriction(0.0)
      , contactMaterialViscousFriction(0.0)
      , contactImpactCaptureVelocity(0.0)
      , contactStictionTransitionVelocity(0.0)
      , dynamicsWorld(nullptr)
      , stepTimeDouble(0.0)
{
  // Instantiate the Multibody System
  // Instantiate the Simbody Matter Subsystem
  // Instantiate the Simbody General Force Subsystem

  this->simbodyPhysicsInitialized = false;
  this->simbodyPhysicsStepped = false;
}

//////////////////////////////////////////////////
SimbodyPhysics::~SimbodyPhysics()
{
}

//////////////////////////////////////////////////
ModelPtr SimbodyPhysics::CreateModel(BasePtr _parent)
{
  // set physics as uninitialized
  this->simbodyPhysicsInitialized = false;

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
  this->solverType = "elastic_foundation";

  /// \TODO: get from sdf for simbody physics
  this->integratorType = "semi_explicit_euler";

  if (this->integratorType == "rk_merson")
    this->integ = new SimTK::RungeKuttaMersonIntegrator(system);
  else if (this->integratorType == "rk3")
    this->integ = new SimTK::RungeKutta3Integrator(system);
  else if (this->integratorType == "rk2")
    this->integ = new SimTK::RungeKutta2Integrator(system);
  else if (this->integratorType == "semi_explicit_euler")
    this->integ = new SimTK::SemiExplicitEuler2Integrator(system);
  else
  {
    gzerr << "type not specified, using SemiExplicitEuler2Integrator.\n";
    this->integ = new SimTK::SemiExplicitEuler2Integrator(system);
  }

  this->stepTimeDouble = this->GetMaxStepSize();

  sdf::ElementPtr simbodyElem = this->sdf->GetElement("simbody");

  // Set integrator accuracy (measured with Richardson Extrapolation)
  this->integ->setAccuracy(
    simbodyElem->Get<double>("accuracy"));

  // Set stiction max slip velocity to make it less stiff.
  this->contact.setTransitionVelocity(
    simbodyElem->Get<double>("max_transient_velocity"));

  sdf::ElementPtr simbodyContactElem = simbodyElem->GetElement("contact");

  // system wide contact properties, assigned in AddCollisionsToLink()
  this->contactMaterialStiffness =
    simbodyContactElem->Get<double>("stiffness");
  this->contactMaterialDissipation =
    simbodyContactElem->Get<double>("dissipation");
  this->contactMaterialStaticFriction =
    simbodyContactElem->Get<double>("static_friction");
  this->contactMaterialDynamicFriction =
    simbodyContactElem->Get<double>("dynamic_friction");
  this->contactMaterialViscousFriction =
    simbodyContactElem->Get<double>("viscous_friction");

  // below are not used yet, but should work it into the system
  this->contactMaterialPlasticCoefRestitution =
    simbodyContactElem->Get<double>("plastic_coef_restitution");
  this->contactMaterialPlasticImpactVelocity =
    simbodyContactElem->Get<double>("plastic_impact_velocity");
  this->contactImpactCaptureVelocity =
    simbodyContactElem->Get<double>("override_impact_capture_velocity");
  this->contactStictionTransitionVelocity =
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
    physicsMsg.set_min_step_size(this->GetMaxStepSize());
    physicsMsg.set_enable_physics(this->world->PhysicsEnabled());

    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
      msgs::Convert(this->world->MagneticField()));
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
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
    this->world->SetPhysicsEnabled(_msg->enable_physics());

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
    this->integ->setAccuracy(_msg->simbody().accuracy());
  }

  // Set stiction max slip velocity to make it less stiff.
  if (_msg->has_max_transient_velocity())
  {
    this->contact.setTransitionVelocity(
    _msg->simbody().max_transient_velocity());
  }
  */

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
void SimbodyPhysics::Reset()
{
  this->integ->initialize(this->system.getDefaultState());

  // restore potentially user run-time modified gravity
  this->SetGravity(this->world->Gravity());
}

//////////////////////////////////////////////////
void SimbodyPhysics::Init()
{
  this->simbodyPhysicsInitialized = true;
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitModel(const physics::ModelPtr _model)
{
  // Before building a new system, transfer all joints in existing
  // models, save Simbody joint states in Gazebo Model.
  const SimTK::State& currentState = this->integ->getState();
  double stateTime = 0;
  bool simbodyStateSaved = false;

  if (currentState.getSystemStage() != SimTK::Stage::Empty)
  {
    stateTime = currentState.getTime();
    physics::Model_V models = this->world->Models();
    for (physics::Model_V::iterator mi = models.begin();
         mi != models.end(); ++mi)
    {
      if ((*mi) != _model)
      {
        physics::Joint_V joints = (*mi)->GetJoints();
        for (physics::Joint_V::iterator jx = joints.begin();
             jx != joints.end(); ++jx)
        {
          SimbodyJointPtr simbodyJoint =
            boost::dynamic_pointer_cast<physics::SimbodyJoint>(*jx);
          simbodyJoint->SaveSimbodyState(currentState);
        }

        physics::Link_V links = (*mi)->GetLinks();
        for (physics::Link_V::iterator lx = links.begin();
             lx != links.end(); ++lx)
        {
          SimbodyLinkPtr simbodyLink =
            boost::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
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
      SimTK::MultibodyGraphMaker mbgraph;
      this->CreateMultibodyGraph(mbgraph, _model);
      // Optional: dump the graph to stdout for debugging or curiosity.
      // mbgraph.dumpGraph(gzdbg);

      SimbodyPhysics::AddDynamicModelToSimbodySystem(mbgraph, _model);
    }
  }
  catch(const std::exception& e)
  {
    gzthrow(std::string("Simbody build EXCEPTION: ") + e.what());
  }

  try
  {
    //------------------------ CREATE SIMBODY SYSTEM ---------------------------
    // Create a Simbody System and populate it with Subsystems we'll need.
    SimbodyPhysics::InitSimbodySystem();
  }
  catch(const std::exception& e)
  {
    gzthrow(std::string("Simbody init EXCEPTION: ") + e.what());
  }

  SimTK::State state = this->system.realizeTopology();

  // Restore Gazebo saved Joint states
  // back into Simbody state.
  if (simbodyStateSaved)
  {
    // set/retsore state time.
    state.setTime(stateTime);

    physics::Model_V models = this->world->Models();
    for (physics::Model_V::iterator mi = models.begin();
         mi != models.end(); ++mi)
    {
      physics::Joint_V joints = (*mi)->GetJoints();
      for (physics::Joint_V::iterator jx = joints.begin();
           jx != joints.end(); ++jx)
      {
        SimbodyJointPtr simbodyJoint =
          boost::dynamic_pointer_cast<physics::SimbodyJoint>(*jx);
        simbodyJoint->RestoreSimbodyState(state);
      }
      physics::Link_V links = (*mi)->GetLinks();
      for (physics::Link_V::iterator lx = links.begin();
           lx != links.end(); ++lx)
      {
        SimbodyLinkPtr simbodyLink =
          boost::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
        simbodyLink->RestoreSimbodyState(state);
      }
    }
  }

  // initialize integrator from state
  this->integ->initialize(state);

  // mark links as initialized
  Link_V links = _model->GetLinks();
  for (Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    physics::SimbodyLinkPtr simbodyLink =
      boost::dynamic_pointer_cast<physics::SimbodyLink>(*li);
    if (simbodyLink)
      simbodyLink->physicsInitialized = true;
    else
      gzerr << "failed to cast link [" << (*li)->GetName()
            << "] as simbody link\n";
  }

  // mark joints as initialized
  physics::Joint_V joints = _model->GetJoints();
  for (physics::Joint_V::iterator ji = joints.begin();
       ji != joints.end(); ++ji)
  {
    SimbodyJointPtr simbodyJoint =
      boost::dynamic_pointer_cast<SimbodyJoint>(*ji);
    if (simbodyJoint)
      simbodyJoint->physicsInitialized = true;
    else
      gzerr << "simbodyJoint [" << (*ji)->GetName()
            << "]is not a SimbodyJointPtr\n";
  }

  this->simbodyPhysicsInitialized = true;
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitForThread()
{
  IGN_PROFILE_THREAD_NAME("SimbodyPhysics");
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdateCollision()
{
  IGN_PROFILE("SimbodyPhysics::UpdateCollision");
  IGN_PROFILE_BEGIN("UpdateCollision");
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  this->contactManager->ResetCount();

  // Get all contacts from Simbody
  const SimTK::State &state = this->integ->getState();

  // The tracker cannot generate a snapshot without a subsystem
  if (state.getNumSubsystems() == 0)
    return;

  // get contact snapshot
  const SimTK::ContactSnapshot &contactSnapshot =
    this->tracker.getActiveContacts(state);

  int numc = contactSnapshot.getNumContacts();

  int count = 0;
  for (int j = 0; j < numc; ++j)
  {
    // get contact stuff from Simbody
    const SimTK::Contact &simbodyContact = contactSnapshot.getContact(j);
    {
      SimTK::ContactSurfaceIndex csi1 = simbodyContact.getSurface1();
      SimTK::ContactSurfaceIndex csi2 = simbodyContact.getSurface2();
      const SimTK::ContactSurface &cs1 = this->tracker.getContactSurface(csi1);
      const SimTK::ContactSurface &cs2 = this->tracker.getContactSurface(csi2);

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
      physics::Model_V models = this->world->Models();
      for (physics::Model_V::iterator mi = models.begin();
           mi != models.end(); ++mi)
      {
        physics::Link_V links = (*mi)->GetLinks();
        for (Link_V::iterator li = links.begin(); li != links.end(); ++li)
        {
          Collision_V collisions = (*li)->GetCollisions();
          for (Collision_V::iterator ci = collisions.begin();
               ci != collisions.end(); ++ci)
          {
            /// compare SimbodyCollision::GetCollisionShape() to
            /// ContactGeometry from SimTK::ContactForce
            SimbodyCollisionPtr sc =
              boost::dynamic_pointer_cast<physics::SimbodyCollision>(*ci);
            if (sc->GetCollisionShape() == &cg1)
            {
              collision1 = (*ci).get();
              link1 = (*li);
            }
            else if (sc->GetCollisionShape() == &cg2)
            {
              collision2 = (*ci).get();
              link2 = (*li);
            }
          }
        }
      }

      // add contacts to the manager. This will return nullptr if no one is
      // listening for contact information.
      Contact *contactFeedback = this->contactManager->NewContact(collision1,
          collision2, this->world->SimTime());

      if (contactFeedback)
      {
        // const bool useContactPatch = true;
        // if (useContactPatch)
        {
          // get contact patch to get detailed contacts
          // see https://github.com/simbody/simbody/blob/master/examples/ExampleContactPlayground.cpp#L110
          SimTK::ContactPatch patch;
          this->system.realize(state, SimTK::Stage::Velocity);
          const bool found =
             this->contact.calcContactPatchDetailsById(
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

              // actually don't need to do this? confirm that
              //    everything is in the world frame!
              // /// \TODO: rotate it into link 1 frame, there must be
              // /// a clean way to do this in simbody...
              // /// my gazebo way of rotating frames for now, to replace with
              // /// clean simbody function calls.
              // /// rotation from link2 to link1 frame specified in link2 frame
              // math::Quaternion rot21 = (pose1 - pose2).rot;
              // t1cg = SimbodyPhysics::Vector3ToVec3(
              //   rot21.RotateVectorReverse(
              //   SimbodyPhysics::Vec3ToVector3(t1cg)));
              // f1cg = SimbodyPhysics::Vector3ToVec3(
              //   rot21.RotateVectorReverse(
              //   SimbodyPhysics::Vec3ToVector3(f1cg)));

              // gzerr << "numc: " << j << "\n";
              // gzerr << "count: " << count << "\n";
              // gzerr << "index: " << i << "\n";
              // gzerr << "offset 2: " << detail.getContactPoint() << "\n";
              // gzerr << "s2: " << s2 << "\n";
              // gzerr << "s2cg: " << s2cg << "\n";
              // gzerr << "f2cg: " << f2cg << "\n";
              // gzerr << "t2cg: " << t2cg << "\n";
              // gzerr << "offset 1: " << detail.getContactPoint() << "\n";
              // gzerr << "s1cg: " << s1cg << "\n";
              // gzerr << "f1cg: " << f1cg << "\n";
              // gzerr << "t1cg: " << t1cg << "\n";

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
        // else  // use single ContactForce
        // {
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
        // }
      }
    }
  }
  IGN_PROFILE_END();
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdatePhysics()
{
  IGN_PROFILE("SimbodyPhysics::UpdatePhysics");
  IGN_PROFILE_BEGIN("UpdatePhysics");

  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  common::Time currTime =  this->world->RealTime();

  // Simbody cannot step the integrator without a subsystem
  const SimTK::State &s = this->integ->getState();
  if (s.getNumSubsystems() == 0)
    return;

  bool trying = true;
  while (trying && integ->getTime() < this->world->SimTime().Double())
  {
    try
    {
      this->integ->stepTo(this->world->SimTime().Double(),
                          this->world->SimTime().Double());
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
  //       << "] dt [" << this->stepTimeDouble
  //       << "] t [" << this->world->SimTime().Double()
  //       << "]\n";
  // this->lastUpdateTime = currTime;

  // pushing new entity pose into dirtyPoses for visualization
  physics::Model_V models = this->world->Models();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = (*mi)->GetLinks();
    for (physics::Link_V::iterator lx = links.begin();
         lx != links.end(); ++lx)
    {
      physics::SimbodyLinkPtr simbodyLink =
        boost::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
      auto pose = SimbodyPhysics::Transform2PoseIgn(
        simbodyLink->masterMobod.getBodyTransform(s));
      simbodyLink->SetDirtyPose(pose);
      this->world->dataPtr->dirtyPoses.push_back(
        boost::static_pointer_cast<Entity>(*lx).get());
    }

    physics::Joint_V joints = (*mi)->GetJoints();
    for (physics::Joint_V::iterator jx = joints.begin();
         jx != joints.end(); ++jx)
    {
      SimbodyJointPtr simbodyJoint =
        boost::dynamic_pointer_cast<physics::SimbodyJoint>(*jx);
      simbodyJoint->CacheForceTorque();
    }
  }

  // FIXME:  this needs to happen before forces are applied for the next step
  // FIXME:  but after we've gotten everything from current state
  this->discreteForces.clearAllForces(this->integ->updAdvancedState());
  IGN_PROFILE_END();
}

//////////////////////////////////////////////////
void SimbodyPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
LinkPtr SimbodyPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == nullptr)
    gzthrow("Link must have a parent\n");

  SimbodyLinkPtr link(new SimbodyLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr SimbodyPhysics::CreateCollision(const std::string &_type,
                                            LinkPtr _parent)
{
  SimbodyCollisionPtr collision(new SimbodyCollision(_parent));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_parent->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr SimbodyPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  SimbodyCollisionPtr collision =
    boost::dynamic_pointer_cast<SimbodyCollision>(_collision);

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
    if (_collision)
      shape.reset(new SimbodyRayShape(_collision));
    else
      shape.reset(new SimbodyRayShape(this->world->Physics()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

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
    joint.reset(new SimbodyHingeJoint(this->dynamicsWorld, _parent));
  else if (_type == "universal")
    joint.reset(new SimbodyUniversalJoint(this->dynamicsWorld, _parent));
  else if (_type == "ball")
    joint.reset(new SimbodyBallJoint(this->dynamicsWorld, _parent));
  else if (_type == "prismatic")
    joint.reset(new SimbodySliderJoint(this->dynamicsWorld, _parent));
  else if (_type == "revolute2")
    joint.reset(new SimbodyHinge2Joint(this->dynamicsWorld, _parent));
  else if (_type == "screw")
    joint.reset(new SimbodyScrewJoint(this->dynamicsWorld, _parent));
  else if (_type == "fixed")
    joint.reset(new SimbodyFixedJoint(this->dynamicsWorld, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->world->SetGravitySDF(_gravity);

  {
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
    if (this->simbodyPhysicsInitialized && this->world->ModelCount() > 0)
      this->gravity.setGravityVector(this->integ->updAdvancedState(),
         SimbodyPhysics::Vector3ToVec3(_gravity));
    else
      this->gravity.setDefaultGravityVector(
        SimbodyPhysics::Vector3ToVec3(_gravity));
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
  _mbgraph.addJointType(GetTypeString(physics::Base::HINGE_JOINT),  1);
  _mbgraph.addJointType(GetTypeString(physics::Base::HINGE2_JOINT), 2);
  _mbgraph.addJointType(GetTypeString(physics::Base::SLIDER_JOINT), 1);
  _mbgraph.addJointType(GetTypeString(physics::Base::UNIVERSAL_JOINT), 2);
  _mbgraph.addJointType(GetTypeString(physics::Base::SCREW_JOINT), 1);
  _mbgraph.addJointType(GetTypeString(physics::Base::FIXED_JOINT), 0);

  // Simbody has a Ball constraint that is a good choice if you need to
  // break a loop at a ball joint.
  // _mbgraph.addJointType(GetTypeString(physics::Base::BALL_JOINT), 3, true);
  // skip loop joints for now
  _mbgraph.addJointType(GetTypeString(physics::Base::BALL_JOINT), 3, false);

  // Step 2: Tell it about all the links we read from the input file,
  // starting with world, and provide a reference pointer.
  _mbgraph.addBody("world", SimTK::Infinity,
                  false);

  physics::Link_V links = _model->GetLinks();
  for (physics::Link_V::iterator li = links.begin();
       li != links.end(); ++li)
  {
    SimbodyLinkPtr simbodyLink = boost::dynamic_pointer_cast<SimbodyLink>(*li);

    // gzerr << "debug : " << (*li)->GetName() << "\n";

    if (simbodyLink)
    {
      _mbgraph.addBody((*li)->GetName(), (*li)->GetInertial()->Mass(),
                      simbodyLink->mustBeBaseLink, (*li).get());
    }
    else
    {
      gzerr << "simbodyLink [" << (*li)->GetName()
            << "]is not a SimbodyLinkPtr\n";
    }
  }

  // Step 3: Tell it about all the joints we read from the input file,
  // and provide a reference pointer.
  physics::Joint_V joints = _model->GetJoints();
  for (physics::Joint_V::iterator ji = joints.begin();
       ji != joints.end(); ++ji)
  {
    SimbodyJointPtr simbodyJoint =
      boost::dynamic_pointer_cast<SimbodyJoint>(*ji);
    if (simbodyJoint)
      if ((*ji)->GetParent() && (*ji)->GetChild())
        _mbgraph.addJoint((*ji)->GetName(), GetTypeString((*ji)->GetType()),
           (*ji)->GetParent()->GetName(), (*ji)->GetChild()->GetName(),
                            simbodyJoint->mustBreakLoopHere, (*ji).get());
      else if ((*ji)->GetChild())
        _mbgraph.addJoint((*ji)->GetName(), GetTypeString((*ji)->GetType()),
           "world", (*ji)->GetChild()->GetName(),
                            simbodyJoint->mustBreakLoopHere, (*ji).get());
      else
        gzerr << "simbodyJoint [" << (*ji)->GetName()
              << "] does not have a valid child link, which is required\n";
    else
      gzerr << "simbodyJoint [" << (*ji)->GetName()
            << "]is not a SimbodyJointPtr\n";
  }

  // Setp 4. Generate the multibody graph.
  _mbgraph.generateGraph();
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitSimbodySystem()
{
  // Set stiction max slip velocity to make it less stiff.
  // this->contact.setTransitionVelocity(0.01);  // now done in Load using sdf

  // Specify gravity (read in above from world).
  if (!ignition::math::equal(this->world->Gravity().Length(), 0.0))
    this->gravity.setDefaultGravityVector(
      SimbodyPhysics::Vector3ToVec3(this->world->Gravity()));
  else
    this->gravity.setDefaultMagnitude(0.0);
}

//////////////////////////////////////////////////
void SimbodyPhysics::AddStaticModelToSimbodySystem(
    const physics::ModelPtr _model)
{
  physics::Link_V links = _model->GetLinks();
  for (physics::Link_V::iterator li = links.begin();
       li != links.end(); ++li)
  {
    SimbodyLinkPtr simbodyLink = boost::dynamic_pointer_cast<SimbodyLink>(*li);
    if (simbodyLink)
    {
      this->AddCollisionsToLink(simbodyLink.get(), this->matter.updGround(),
        SimTK::ContactCliqueId());
      simbodyLink->masterMobod = this->matter.updGround();
    }
    else
      gzerr << "simbodyLink [" << (*li)->GetName()
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
  SimTK::ContactCliqueId modelClique = SimTK::ContactSurface::createNewContactClique();

  // Will specify explicitly when needed
  // Record the MobilizedBody for the World link.
  // model.links.updLink(0).masterMobod = this->matter.Ground();

  // Run through all the mobilizers in the multibody graph, adding a Simbody
  // MobilizedBody for each one. Also add visual and collision geometry to the
  // bodies when they are mobilized.
  for (int mobNum = 0; mobNum < _mbgraph.getNumMobilizers(); ++mobNum)
  {
    // Get a mobilizer from the graph, then extract its corresponding
    // joint and bodies. Note that these don't necessarily have equivalents
    // in the GazeboLink and GazeboJoint inputs.
    const SimTK::MultibodyGraphMaker::Mobilizer& mob = _mbgraph.getMobilizer(mobNum);
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

    const SimTK::MassProperties massProps =
        gzOutb->GetEffectiveMassProps(mob.getNumFragments());

    // debug
    // if (gzInb)
    //   gzerr << "debug: Inb: " << gzInb->GetName() << "\n";
    // if (gzOutb)
    //   gzerr << "debug: Outb: " << gzOutb->GetName()
    //         << " mass: " << gzOutb->GetInertial()->GetMass()
    //         << " efm: " << massProps
    //         << "\n";

    // This will reference the new mobilized body once we create it.
    SimTK::MobilizedBody mobod;

    SimTK::MobilizedBody parentMobod =
      gzInb == nullptr ? this->matter.Ground() : gzInb->masterMobod;

    if (mob.isAddedBaseMobilizer())
    {
      // There is no corresponding Gazebo joint for this mobilizer.
      // Create the joint and set its default position to be the default
      // pose of the base link relative to the Ground frame.
      // Currently only `free` is allowed, we may add more types later
      GZ_ASSERT(type == "free", "type is not 'free', not allowed.");
      if (type == "free")
      {
        SimTK::MobilizedBody::Free freeJoint(
            parentMobod,  SimTK::Transform(),
            massProps,    SimTK::Transform());

        SimTK::Transform inboard_X_ML;
        if (gzInb == nullptr)
        {
          // GZ_ASSERT(gzOutb, "must be here");
          physics::ModelPtr model = gzOutb->GetParentModel();
          inboard_X_ML =
            ~SimbodyPhysics::Pose2Transform(model->WorldPose());
        }
        else
          inboard_X_ML =
            SimbodyPhysics::Pose2Transform(gzInb->RelativePose());

        SimTK::Transform outboard_X_ML =
          SimbodyPhysics::Pose2Transform(gzOutb->RelativePose());

        // defX_ML link frame specified in model frame
        freeJoint.setDefaultTransform(~inboard_X_ML*outboard_X_ML);
        mobod = freeJoint;
      }
    }
    else
    {
      // This mobilizer does correspond to one of the input joints.
      // note: do not use boost shared pointer here, on scope out the
      // original pointer get scrambled
      SimbodyJoint* gzJoint = static_cast<SimbodyJoint*>(mob.getJointRef());
      const bool isReversed = mob.isReversedFromJoint();

      // Find inboard and outboard frames for the mobilizer; these are
      // parent and child frames or the reverse.

      const SimTK::Transform& X_IF0 = isReversed ? gzJoint->xCB : gzJoint->xPA;
      const SimTK::Transform& X_OM0 = isReversed ? gzJoint->xPA : gzJoint->xCB;

      const SimTK::MobilizedBody::Direction direction =
          isReversed ? SimTK::MobilizedBody::Reverse : SimTK::MobilizedBody::Forward;

      if (type == "free")
      {
        SimTK::MobilizedBody::Free freeJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        SimTK::Transform defX_FM = isReversed ? SimTK::Transform(~gzJoint->defxAB)
                                       : gzJoint->defxAB;
        freeJoint.setDefaultTransform(defX_FM);
        mobod = freeJoint;
      }
      else if (type == "screw")
      {
        SimTK::UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(
            gzJoint->LocalAxis(0))));

        double pitch =
          dynamic_cast<physics::SimbodyScrewJoint*>(gzJoint)->GetThreadPitch(0);

        if (ignition::math::equal(pitch, 0.0))
        {
          gzerr << "thread pitch should not be zero (joint is a slider?)"
                << " using pitch = 1.0e6\n";
          pitch = 1.0e6;
        }

        // Simbody's screw joint axis (both rotation and translation) is along Z
        SimTK::Rotation R_JZ(axis, SimTK::ZAxis);
        SimTK::Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
        SimTK::Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
        SimTK::MobilizedBody::Screw screwJoint(
            parentMobod,      X_IF,
            massProps,        X_OM,
            -1.0/pitch,
            direction);
        mobod = screwJoint;

        gzdbg << "Setting limitForce[0] for [" << gzJoint->GetName() << "]\n";

        double low = gzJoint->LowerLimit(0u);
        double high = gzJoint->UpperLimit(0u);

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->limitForce[0] =
          SimTK::Force::MobilityLinearStop(this->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->GetStopStiffness(0),
          gzJoint->GetStopDissipation(0), low, high);

        // gzdbg << "SimbodyPhysics SetDamping ("
        //       << gzJoint->GetDampingCoefficient()
        //       << ")\n";
        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->damper[0] =
          SimTK::Force::MobilityLinearDamper(this->forces, mobod, 0,
                                   gzJoint->GetDamping(0));

        // add spring (stiffness proportional to mass)
        gzJoint->spring[0] =
          SimTK::Force::MobilityLinearSpring(this->forces, mobod, 0,
            gzJoint->GetStiffness(0),
            gzJoint->GetSpringReferencePosition(0));
      }
      else if (type == "universal")
      {
        SimTK::UnitVec3 axis1(SimbodyPhysics::Vector3ToVec3(
          gzJoint->AxisFrameOffset(0).RotateVector(
          gzJoint->LocalAxis(UniversalJoint<Joint>::AXIS_PARENT))));
        /// \TODO: check if this is right, or AxisFrameOffset(1) is needed.
        SimTK::UnitVec3 axis2(SimbodyPhysics::Vector3ToVec3(
          gzJoint->AxisFrameOffset(0).RotateVector(
          gzJoint->LocalAxis(UniversalJoint<Joint>::AXIS_CHILD))));

        // Simbody's univeral joint is along axis1=Y and axis2=X
        // note X and Y are reversed because Simbody defines universal joint
        // rotation in body-fixed frames, whereas Gazebo/ODE uses space-fixed
        // frames.
        SimTK::Rotation R_JF(axis1, SimTK::XAxis, axis2, SimTK::YAxis);
        SimTK::Transform X_IF(X_IF0.R()*R_JF, X_IF0.p());
        SimTK::Transform X_OM(X_OM0.R()*R_JF, X_OM0.p());
        SimTK::MobilizedBody::Universal uJoint(
            parentMobod,      X_IF,
            massProps,        X_OM,
            direction);
        mobod = uJoint;

        for (unsigned int nj = 0; nj < 2; ++nj)
        {
          double low = gzJoint->LowerLimit(nj);
          double high = gzJoint->UpperLimit(nj);

          // initialize stop stiffness and dissipation from joint parameters
          gzJoint->limitForce[nj] =
            SimTK::Force::MobilityLinearStop(this->forces, mobod,
            SimTK::MobilizerQIndex(nj), gzJoint->GetStopStiffness(nj),
            gzJoint->GetStopDissipation(nj), low, high);

          // gzdbg << "stop stiffness [" << gzJoint->GetStopStiffness(nj)
          //       << "] low [" << low
          //       << "] high [" << high
          //       << "]\n";

          // gzdbg << "SimbodyPhysics SetDamping ("
          //       << gzJoint->GetDampingCoefficient()
          //       << ")\n";
          // Create a damper for every joint even if damping coefficient
          // is zero.  This will allow user to change damping coefficients
          // on the fly.
          gzJoint->damper[nj] =
            SimTK::Force::MobilityLinearDamper(this->forces, mobod, nj,
                                     gzJoint->GetDamping(nj));
          // add spring (stiffness proportional to mass)
          gzJoint->spring[nj] =
            SimTK::Force::MobilityLinearSpring(this->forces, mobod, nj,
              gzJoint->GetStiffness(nj),
              gzJoint->GetSpringReferencePosition(nj));
        }
      }
      else if (type == "revolute")
      {
        // rotation from axis frame to child link frame
        // simbody assumes links are in child link frame, but gazebo
        // sdf 1.4 and earlier assumes joint axis are defined in model frame.
        // Use function Joint::GetAxisFrame() to remedy this situation.
        // Joint::GetAxisFrame() returns the frame joint axis is defined:
        // either model frame or child link frame.
        // simbody always assumes axis is specified in the child link frame.
        // \TODO: come up with a test case where we might need to
        // flip transform based on isReversed flag.
        SimTK::UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(
            gzJoint->LocalAxis(0))));

        // gzerr << "[" << gzJoint->AxisFrameOffset(0).Euler()
        //       << "] ["
        //       << gzJoint->AxisFrameOffset(0).RotateVector(
        //          gzJoint->LocalAxis(0)) << "]\n";

        // Simbody's pin is along Z
        SimTK::Rotation R_JZ(axis, SimTK::ZAxis);
        SimTK::Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
        SimTK::Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
        SimTK::MobilizedBody::Pin pinJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = pinJoint;

        double low = gzJoint->LowerLimit(0u);
        double high = gzJoint->UpperLimit(0u);

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->limitForce[0] =
          SimTK::Force::MobilityLinearStop(this->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->GetStopStiffness(0),
          gzJoint->GetStopDissipation(0), low, high);

        // gzdbg << "SimbodyPhysics SetDamping ("
        //       << gzJoint->GetDampingCoefficient()
        //       << ")\n";
        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->damper[0] =
          SimTK::Force::MobilityLinearDamper(this->forces, mobod, 0,
                                   gzJoint->GetDamping(0));

        // add spring (stiffness proportional to mass)
        gzJoint->spring[0] =
          SimTK::Force::MobilityLinearSpring(this->forces, mobod, 0,
            gzJoint->GetStiffness(0),
            gzJoint->GetSpringReferencePosition(0));
      }
      else if (type == "prismatic")
      {
        SimTK::UnitVec3 axis(SimbodyPhysics::Vector3ToVec3(
            gzJoint->AxisFrameOffset(0).RotateVector(
            gzJoint->LocalAxis(0))));

        // Simbody's slider is along X
        SimTK::Rotation R_JX(axis, SimTK::XAxis);
        SimTK::Transform X_IF(X_IF0.R()*R_JX, X_IF0.p());
        SimTK::Transform X_OM(X_OM0.R()*R_JX, X_OM0.p());
        SimTK::MobilizedBody::Slider sliderJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = sliderJoint;

        double low = gzJoint->LowerLimit(0u);
        double high = gzJoint->UpperLimit(0u);

        // initialize stop stiffness and dissipation from joint parameters
        gzJoint->limitForce[0] =
          SimTK::Force::MobilityLinearStop(this->forces, mobod,
          SimTK::MobilizerQIndex(0), gzJoint->GetStopStiffness(0),
          gzJoint->GetStopDissipation(0), low, high);

        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->damper[0] =
          SimTK::Force::MobilityLinearDamper(this->forces, mobod, 0,
                                   gzJoint->GetDamping(0));

        // add spring (stiffness proportional to mass)
        gzJoint->spring[0] =
          SimTK::Force::MobilityLinearSpring(this->forces, mobod, 0,
            gzJoint->GetStiffness(0),
            gzJoint->GetSpringReferencePosition(0));
      }
      else if (type == "ball")
      {
        SimTK::MobilizedBody::Ball ballJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        SimTK::Rotation defR_FM = isReversed
            ? SimTK::Rotation(~gzJoint->defxAB.R())
            : gzJoint->defxAB.R();
        ballJoint.setDefaultRotation(defR_FM);
        mobod = ballJoint;
      }
      else if (type == "fixed")
      {
        SimTK::MobilizedBody::Weld fixedJoint(
            parentMobod,  X_IF0,
            massProps,    X_OM0);
        mobod = fixedJoint;
      }
      else
      {
        gzerr << "Simbody joint type [" << type << "] not implemented.\n";
      }

      // Created a mobilizer that corresponds to gzJoint. Keep track.
      gzJoint->mobod = mobod;
      gzJoint->isReversed = isReversed;
    }

    // Link gzOutb has been mobilized; keep track for later.
    if (isSlave)
      gzOutb->slaveMobods.push_back(mobod);
    else
      gzOutb->masterMobod = mobod;

    // A mobilizer has been created; now add the collision
    // geometry for the new mobilized body.
    this->AddCollisionsToLink(gzOutb, mobod, modelClique);
  }

  // Weld the slaves to their masters.
  physics::Model_V models = this->world->Models();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = (*mi)->GetLinks();
    for (physics::Link_V::iterator lx = links.begin();
         lx != links.end(); ++lx)
    {
      physics::SimbodyLinkPtr link =
        boost::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
      if (link->slaveMobods.empty()) continue;
      for (unsigned i = 0; i < link->slaveMobods.size(); ++i)
      {
        SimTK::Constraint::Weld weld(link->masterMobod, link->slaveMobods[i]);

        // in case we want to know later
        link->slaveWelds.push_back(weld);
      }
    }
  }

  //   leave out optimization
  // // Add the loop joints if any.
  // for (int lcx=0; lcx < _mbgraph.getNumLoopConstraints(); ++lcx) {
  //     const SimTK::MultibodyGraphMaker::LoopConstraint& loop =
  //         _mbgraph.getLoopConstraint(lcx);

  //     SimbodyJointPtr joint(loop.getJointRef());
  //     SimbodyLinkPtr  parent(loop.getParentBodyRef());
  //     SimbodyLinkPtr  child(loop.getChildBodyRef());

  //     if (joint.type == "weld") {
  //         SimTK::Constraint::Weld weld(parent.masterMobod, joint.xPA,
  //                               child.masterMobod,  joint.xCB);
  //         joint.constraint = weld;
  //     } else if (joint.type == "ball") {
  //         SimTK::Constraint::Ball ball(parent.masterMobod, joint.xPA.p(),
  //                               child.masterMobod,  joint.xCB.p());
  //         joint.constraint = ball;
  //     } else if (joint.type == "free") {
  //         // A "free" loop constraint is no constraint at all so we can
  //         // just ignore it. It might be more convenient if there were
  //         // a 0-constraint SimTK::Constraint::Free, just as there is a 0-mobility
  //         // SimTK::MobilizedBody::Weld.
  //     } else
  //         throw std::runtime_error(
  //             "Unrecognized loop constraint type '" + joint.type + "'.");
  // }
}

std::string SimbodyPhysics::GetTypeString(physics::Base::EntityType _type)
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
void SimbodyPhysics::SetSeed(uint32_t /*_seed*/)
{
  gzerr << "SimbodyPhysics::SetSeed not implemented\n";
}

/////////////////////////////////////////////////
void SimbodyPhysics::AddCollisionsToLink(const physics::SimbodyLink *_link,
  SimTK::MobilizedBody &_mobod, SimTK::ContactCliqueId _modelClique)
{
  // TODO: Edit physics::Surface class to support these properties
  // Define a material to use for contact. This is not very stiff.
  // use stiffness of 1e8 and dissipation of 1000.0 to approximate inelastic
  // collision. but 1e6 and 10 seems sufficient when TransitionVelocity is
  // reduced from 0.1 to 0.01
  SimTK::ContactMaterial material(this->contactMaterialStiffness,
                                  this->contactMaterialDissipation,
                                  this->contactMaterialStaticFriction,
                                  this->contactMaterialDynamicFriction,
                                  this->contactMaterialViscousFriction);
  // Debug: works for SpawnDrop
  // SimTK::ContactMaterial material(1e6,   // stiffness
  //                                 10.0,  // dissipation
  //                                 0.7,   // mu_static
  //                                 0.5,   // mu_dynamic
  //                                 0.5);  // mu_viscous

  bool addModelClique = _modelClique.isValid() && !_link->GetSelfCollide();

  // COLLISION
  Collision_V collisions =  _link->GetCollisions();
  for (Collision_V::iterator ci =  collisions.begin();
                             ci !=  collisions.end(); ++ci)
  {
    SimTK::Transform X_LC =
      SimbodyPhysics::Pose2Transform((*ci)->RelativePose());

    // use pointer to store CollisionGeometry
    SimbodyCollisionPtr sc =
      boost::dynamic_pointer_cast<physics::SimbodyCollision>(*ci);

    switch ((*ci)->GetShapeType() & (~physics::Entity::SHAPE))
    {
      case physics::Entity::PLANE_SHAPE:
      {
        boost::shared_ptr<physics::PlaneShape> p =
          boost::dynamic_pointer_cast<physics::PlaneShape>((*ci)->GetShape());

        // by default, simbody HalfSpace normal is in the -X direction
        // rotate it based on normal vector specified by user
        // Create a rotation whos x-axis is in the
        // negative normal vector direction
        SimTK::Vec3 normal = SimbodyPhysics::Vector3ToVec3(p->Normal());
        SimTK::Rotation R_XN(-SimTK::UnitVec3(normal), SimTK::XAxis);

        SimTK::ContactSurface surface(SimTK::ContactGeometry::HalfSpace(), material);

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
        boost::shared_ptr<physics::SphereShape> s =
          boost::dynamic_pointer_cast<physics::SphereShape>((*ci)->GetShape());
        double r = s->GetRadius();
        SimTK::ContactSurface surface(SimTK::ContactGeometry::Sphere(r), material);
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
        boost::shared_ptr<physics::CylinderShape> c =
          boost::dynamic_pointer_cast<physics::CylinderShape>(
          (*ci)->GetShape());
        double r = c->GetRadius();
        double len = c->GetLength();

        // chunky hexagonal shape
        const int resolution = 1;
        const SimTK::PolygonalMesh mesh = SimTK::PolygonalMesh::
            createCylinderMesh(SimTK::ZAxis, r, len/2, resolution);
        const SimTK::ContactGeometry::TriangleMesh triMesh(mesh);
        SimTK::ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // SimTK::Vec3 esz = SimTK::Vec3(r, r, len/2);  // Use ellipsoid instead
        // SimTK::ContactSurface surface(SimTK::ContactGeometry::Ellipsoid(esz),
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
        SimTK::Vec3 hsz = SimbodyPhysics::Vector3ToVec3(
          (boost::dynamic_pointer_cast<physics::BoxShape>(
          (*ci)->GetShape()))->Size())/2;

        /// \TODO: harcoded resolution, make collision resolution
        /// an adjustable parameter (#980)
        // number times to chop the longest side.
        const int resolution = 6;
        // const int resolution = 10 * (int)(max(hsz)/min(hsz) + 0.5);
        const SimTK::PolygonalMesh mesh = SimTK::PolygonalMesh::
            createBrickMesh(hsz, resolution);
        const SimTK::ContactGeometry::TriangleMesh triMesh(mesh);
        SimTK::ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // SimTK::ContactSurface surface(SimTK::ContactGeometry::Ellipsoid(hsz),
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
        gzerr << "Collision type [" << (*ci)->GetShapeType()
              << "] unimplemented\n";
        break;
    }
  }
}

/////////////////////////////////////////////////
std::string SimbodyPhysics::GetType() const
{
  return "simbody";
}

/////////////////////////////////////////////////
SimTK::MultibodySystem *SimbodyPhysics::GetDynamicsWorld() const
{
  return this->dynamicsWorld;
}

/////////////////////////////////////////////////
SimTK::Quaternion SimbodyPhysics::QuadToQuad(
    const ignition::math::Quaterniond &_q)
{
  return SimTK::Quaternion(_q.W(), _q.X(), _q.Y(), _q.Z());
}

/////////////////////////////////////////////////
ignition::math::Quaterniond SimbodyPhysics::QuadToQuadIgn(
    const SimTK::Quaternion &_q)
{
  return ignition::math::Quaterniond(_q[0], _q[1], _q[2], _q[3]);
}

/////////////////////////////////////////////////
SimTK::Vec3 SimbodyPhysics::Vector3ToVec3(
    const ignition::math::Vector3d &_v)
{
  return SimTK::Vec3(_v.X(), _v.Y(), _v.Z());
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
SimTK::Transform SimbodyPhysics::GetPose(sdf::ElementPtr _element)
{
  const auto pose = _element->Get<ignition::math::Pose3d>("pose");
  return Pose2Transform(pose);
}

/////////////////////////////////////////////////
std::string SimbodyPhysics::GetTypeString(unsigned int _type)
{
  return GetTypeString(physics::Base::EntityType(_type));
}

//////////////////////////////////////////////////
boost::any SimbodyPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool SimbodyPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  if (_key == "solver_type")
  {
    _value = std::string("Spatial Algebra and Elastic Foundation");
  }
  else if (_key == "integrator_type")
  {
    _value = this->integratorType;
  }
  else if (_key == "accuracy")
  {
    if (this->integ)
      _value = this->integ->getAccuracyInUse();
    else
      _value = 0.0f;
  }
  else if (_key == "max_transient_velocity")
  {
    _value = this->contact.getTransitionVelocity();
  }
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
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
      this->integ->setAccuracy(any_cast<double>(_value));
    }
    else if (_key == "max_transient_velocity")
    {
      this->contact.setTransitionVelocity(any_cast<double>(_value));
    }
    else if (_key == "stiffness")
    {
      this->contactMaterialStiffness = any_cast<double>(_value);
    }
    else if (_key == "dissipation")
    {
      this->contactMaterialDissipation = any_cast<double>(_value);
    }
    else if (_key == "plastic_coef_restitution")
    {
      this->contactMaterialPlasticCoefRestitution =
          any_cast<double>(_value);
    }
    else if (_key == "plastic_impact_velocity")
    {
      this->contactMaterialPlasticImpactVelocity =
          any_cast<double>(_value);
    }
    else if (_key == "static_friction")
    {
      this->contactMaterialStaticFriction = any_cast<double>(_value);
    }
    else if (_key == "dynamic_friction")
    {
      this->contactMaterialDynamicFriction = any_cast<double>(_value);
    }
    else if (_key == "viscous_friction")
    {
      this->contactMaterialViscousFriction = any_cast<double>(_value);
    }
    else if (_key == "override_impact_capture_velocity")
    {
      this->contactMaterialPlasticImpactVelocity =
          any_cast<double>(_value);
    }
    else if (_key == "override_stiction_transition_velocity")
    {
      this->contactImpactCaptureVelocity = any_cast<double>(_value);
    }
    else
    {
      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(std::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ") std::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  return true;
}
