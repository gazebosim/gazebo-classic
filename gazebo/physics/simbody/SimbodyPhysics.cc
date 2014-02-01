/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/physics/simbody/SimbodyRayShape.hh"

#include "gazebo/physics/simbody/SimbodyHingeJoint.hh"
#include "gazebo/physics/simbody/SimbodyUniversalJoint.hh"
#include "gazebo/physics/simbody/SimbodyBallJoint.hh"
#include "gazebo/physics/simbody/SimbodySliderJoint.hh"
#include "gazebo/physics/simbody/SimbodyHinge2Joint.hh"
#include "gazebo/physics/simbody/SimbodyScrewJoint.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/MapShape.hh"

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/simbody/SimbodyPhysics.hh"

typedef boost::shared_ptr<gazebo::physics::SimbodyJoint> SimbodyJointPtr;

using namespace gazebo;
using namespace physics;
using namespace SimTK;

GZ_REGISTER_PHYSICS_ENGINE("simbody", SimbodyPhysics)

//////////////////////////////////////////////////
SimbodyPhysics::SimbodyPhysics(WorldPtr _world)
    : PhysicsEngine(_world), system(), matter(system), forces(system),
      gravity(forces, matter, -SimTK::ZAxis, 0),
      discreteForces(forces, matter),
      tracker(system), contact(system, tracker),  integ(NULL)
{
  // Instantiate the Multibody System
  // Instantiate the Simbody Matter Subsystem
  // Instantiate the Simbody General Force Subsystem

  // Create an integrator
  // this->integ = new SimTK::RungeKuttaMersonIntegrator(system);
  // this->integ = new SimTK::RungeKutta3Integrator(system);
  // this->integ = new SimTK::RungeKutta2Integrator(system);
  this->integ = new SimTK::SemiExplicitEuler2Integrator(system);
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
  this->contactMaterialViscousFriction =
    simbodyContactElem->Get<double>("plastic_coef_restitution");
  this->contactMaterialPlasticCoefRestitution =
    simbodyContactElem->Get<double>("plastic_impact_velocity");
  this->contactMaterialPlasticImpactVelocity =
    simbodyContactElem->Get<double>("override_impact_capture_velocity");
  this->contactImpactCaptureVelocity =
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
    // physicsMsg.set_solver_type(this->stepType);
    // min_step_size is defined but not yet used
    physicsMsg.set_min_step_size(this->GetMaxStepSize());
    // physicsMsg.set_precon_iters(this->GetSORPGSPreconIters());
    // physicsMsg.set_iters(this->GetSORPGSIters());
    physicsMsg.set_enable_physics(this->world->GetEnablePhysicsEngine());
    // physicsMsg.set_sor(this->GetSORPGSW());
    // physicsMsg.set_cfm(this->GetWorldCFM());
    // physicsMsg.set_erp(this->GetWorldERP());
    // physicsMsg.set_contact_max_correcting_vel(
    //     this->GetContactMaxCorrectingVel());
    // physicsMsg.set_contact_surface_layer(this->GetContactSurfaceLayer());
    physicsMsg.mutable_gravity()->CopyFrom(msgs::Convert(this->GetGravity()));
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
  // if (_msg->has_solver_type())
  // {
  //   sdf::ElementPtr solverElem =
  //     this->sdf->GetElement("ode")->GetElement("solver");
  //   if (_msg->solver_type() == "quick")
  //   {
  //     solverElem->GetElement("type")->Set("quick");
  //     this->physicsStepFunc = &dWorldQuickStep;
  //   }
  //   else if (_msg->solver_type() == "world")
  //   {
  //     solverElem->GetElement("type")->Set("world");
  //     this->physicsStepFunc = &dWorldStep;
  //   }
  // }

  // if (_msg->has_precon_iters())
  //   this->SetSORPGSPreconIters(_msg->precon_iters());

  // if (_msg->has_iters())
  //   this->SetSORPGSIters(_msg->iters());

  // if (_msg->has_sor())
  //   this->SetSORPGSW(_msg->sor());

  // if (_msg->has_cfm())
  //   this->SetWorldCFM(_msg->cfm());

  // if (_msg->has_erp())
  //   this->SetWorldERP(_msg->erp());

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

  // if (_msg->has_contact_max_correcting_vel())
  //   this->SetContactMaxCorrectingVel(_msg->contact_max_correcting_vel());

  // if (_msg->has_contact_surface_layer())
  //   this->SetContactSurfaceLayer(_msg->contact_surface_layer());

  if (_msg->has_gravity())
    this->SetGravity(msgs::Convert(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
  {
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());
  }

  if (_msg->has_max_step_size())
  {
    this->SetMaxStepSize(_msg->max_step_size());
  }

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
void SimbodyPhysics::Reset()
{
  this->integ->initialize(this->system.getDefaultState());

  // restore potentially user run-time modified gravity
  this->SetGravity(this->GetGravity());
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
    physics::Model_V models = this->world->GetModels();
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
      MultibodyGraphMaker mbgraph;
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

    physics::Model_V models = this->world->GetModels();
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

  // set gravity mode
  Link_V links = _model->GetLinks();
  for (Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    physics::SimbodyLinkPtr simbodyLink =
      boost::dynamic_pointer_cast<physics::SimbodyLink>(*li);
    if (simbodyLink)
      simbodyLink->SetGravityMode(simbodyLink->GetGravityMode());
    else
      gzerr << "failed to cast link [" << (*li)->GetName()
            << "] as simbody link\n";
  }

  this->system.realize(this->integ->getAdvancedState(), Stage::Velocity);

  // mark links as initialized
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
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  common::Time currTime =  this->world->GetRealTime();


  while (integ->getTime() < this->world->GetSimTime().Double())
  {
    // try
    // {
      this->integ->stepTo(this->world->GetSimTime().Double(),
                         this->world->GetSimTime().Double());
    // }
    // catch (...)
    // {
    //   gzerr << "simbody step failed\n";
    // }
  }

  this->simbodyPhysicsStepped = true;
  const SimTK::State &s = this->integ->getState();

  // debug
  // gzerr << "time [" << s.getTime()
  //       << "] q [" << s.getQ()
  //       << "] u [" << s.getU()
  //       << "] dt [" << this->stepTimeDouble
  //       << "] t [" << this->world->GetSimTime().Double()
  //       << "]\n";
  // this->lastUpdateTime = currTime;

  // pushing new entity pose into dirtyPoses for visualization
  physics::Model_V models = this->world->GetModels();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = (*mi)->GetLinks();
    for (physics::Link_V::iterator lx = links.begin();
         lx != links.end(); ++lx)
    {
      physics::SimbodyLinkPtr simbodyLink =
        boost::dynamic_pointer_cast<physics::SimbodyLink>(*lx);
      math::Pose pose = SimbodyPhysics::Transform2Pose(
        simbodyLink->masterMobod.getBodyTransform(s));
      simbodyLink->SetDirtyPose(pose);
      this->world->dirtyPoses.push_back(
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
}

//////////////////////////////////////////////////
void SimbodyPhysics::Fini()
{
}

//////////////////////////////////////////////////
LinkPtr SimbodyPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
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
  else if (_type == "heightmap")
    shape.reset(new SimbodyHeightmapShape(collision));
  else if (_type == "multiray")
    shape.reset(new SimbodyMultiRayShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new SimbodyRayShape(_collision));
    else
      shape.reset(new SimbodyRayShape(this->world->GetPhysicsEngine()));
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
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->Set(_gravity);

  {
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
    if (this->simbodyPhysicsInitialized)
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
      _mbgraph.addBody((*li)->GetName(), (*li)->GetInertial()->GetMass(),
                      simbodyLink->mustBeBaseLink, (*li).get());
    else
      gzerr << "simbodyLink [" << (*li)->GetName()
            << "]is not a SimbodyLinkPtr\n";
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
  if (!math::equal(this->GetGravity().GetLength(), 0.0))
    this->gravity.setDefaultGravityVector(
      SimbodyPhysics::Vector3ToVec3(this->GetGravity()));
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
        ContactCliqueId());
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
  ContactCliqueId modelClique = ContactSurface::createNewContactClique();

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
    MobilizedBody mobod;

    MobilizedBody parentMobod =
      gzInb == NULL ? this->matter.Ground() : gzInb->masterMobod;

    if (mob.isAddedBaseMobilizer())
    {
      // There is no corresponding Gazebo joint for this mobilizer.
      // Create the joint and set its default position to be the default
      // pose of the base link relative to the Ground frame.
      assert(type == "free");  // May add more types later
      if (type == "free")
      {
        MobilizedBody::Free freeJoint(
            parentMobod,  Transform(),
            massProps,    Transform());

        SimTK::Transform inboard_X_ML;
        if (gzInb == NULL)
        {
          // GZ_ASSERT(gzOutb, "must be here");
          physics::ModelPtr model = gzOutb->GetParentModel();
          inboard_X_ML =
            ~SimbodyPhysics::Pose2Transform(model->GetWorldPose());
        }
        else
          inboard_X_ML =
            SimbodyPhysics::Pose2Transform(gzInb->GetRelativePose());

        SimTK::Transform outboard_X_ML =
          SimbodyPhysics::Pose2Transform(gzOutb->GetRelativePose());

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

      const Transform& X_IF0 = isReversed ? gzJoint->xCB : gzJoint->xPA;
      const Transform& X_OM0 = isReversed ? gzJoint->xPA : gzJoint->xCB;

      const MobilizedBody::Direction direction =
          isReversed ? MobilizedBody::Reverse : MobilizedBody::Forward;

      if (type == "free")
      {
        MobilizedBody::Free freeJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        Transform defX_FM = isReversed ? Transform(~gzJoint->defxAB)
                                       : gzJoint->defxAB;
        freeJoint.setDefaultTransform(defX_FM);
        mobod = freeJoint;
      }
      else if (type == "revolute")
      {
        UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(gzJoint->GetLocalAxis(0)));

        // Simbody's pin is along Z
        Rotation R_JZ(axis, ZAxis);
        Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
        MobilizedBody::Pin pinJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = pinJoint;

        gzdbg << "Setting limitForce for [" << gzJoint->GetName() << "]\n";

        double low = gzJoint->GetLowerLimit(0u).Radian();
        double high = gzJoint->GetUpperLimit(0u).Radian();

        /// \TODO: get these from joint.sdf
        const double stopStiffness = 1.0e8;
        const double stopDissipation = 1.0;

        /// \TODO: make stop stiffness, damping with parameters
        /// FIXME: remove hardcoded values.
        gzJoint->limitForce =
          Force::MobilityLinearStop(this->forces, mobod,
          SimTK::MobilizerQIndex(0), stopStiffness, stopDissipation,
          low, high);

        // gzdbg << "SimbodyPhysics SetDamping ("
        //       << gzJoint->GetDampingCoefficient()
        //       << ")\n";
        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->damper =
          Force::MobilityLinearDamper(this->forces, mobod, 0,
                                   gzJoint->GetDampingCoefficient());

        #ifdef ADD_JOINT_SPRINGS
        // KLUDGE add spring (stiffness proportional to mass)
        Force::MobilityLinearSpring(this->forces, mobod, 0,
                                    30*massProps.getMass(), 0);
        #endif
      }
      else if (type == "prismatic")
      {
        UnitVec3 axis(
          SimbodyPhysics::Vector3ToVec3(gzJoint->GetLocalAxis(0)));

        // Simbody's slider is along X
        Rotation R_JX(axis, XAxis);
        Transform X_IF(X_IF0.R()*R_JX, X_IF0.p());
        Transform X_OM(X_OM0.R()*R_JX, X_OM0.p());
        MobilizedBody::Slider sliderJoint(
            parentMobod,      X_IF,
            massProps,              X_OM,
            direction);
        mobod = sliderJoint;

        double low = gzJoint->GetLowerLimit(0u).Radian();
        double high = gzJoint->GetUpperLimit(0u).Radian();

        /// \TODO: get these from joint.sdf
        const double stopStiffness = 1.0e8;
        const double stopDissipation = 1.0;

        /// \TODO: make stop stiffness, damping with parameters
        /// FIXME: remove hardcoded values.
        gzJoint->limitForce =
          Force::MobilityLinearStop(this->forces, mobod,
          SimTK::MobilizerQIndex(0), stopStiffness, stopDissipation,
          low, high);

        // Create a damper for every joint even if damping coefficient
        // is zero.  This will allow user to change damping coefficients
        // on the fly.
        gzJoint->damper =
          Force::MobilityLinearDamper(this->forces, mobod, 0,
                                   gzJoint->GetDampingCoefficient());

        #ifdef ADD_JOINT_SPRINGS
        // KLUDGE add spring (stiffness proportional to mass)
        Force::MobilityLinearSpring(this->forces, mobod, 0,
                                    30*massProps.getMass(), 0);
        #endif
      }
      else if (type == "ball")
      {
        MobilizedBody::Ball ballJoint(
            parentMobod,  X_IF0,
            massProps,          X_OM0,
            direction);
        Rotation defR_FM = isReversed
            ? Rotation(~gzJoint->defxAB.R())
            : gzJoint->defxAB.R();
        ballJoint.setDefaultRotation(defR_FM);
        mobod = ballJoint;
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
  physics::Model_V models = this->world->GetModels();
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
        Constraint::Weld weld(link->masterMobod, link->slaveMobods[i]);

        // in case we want to know later
        link->slaveWelds.push_back(weld);
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
  //         Constraint::Weld weld(parent.masterMobod, joint.xPA,
  //                               child.masterMobod,  joint.xCB);
  //         joint.constraint = weld;
  //     } else if (joint.type == "ball") {
  //         Constraint::Ball ball(parent.masterMobod, joint.xPA.p(),
  //                               child.masterMobod,  joint.xCB.p());
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
  MobilizedBody &_mobod, ContactCliqueId _modelClique)
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
    Transform X_LC =
      SimbodyPhysics::Pose2Transform((*ci)->GetRelativePose());

    switch ((*ci)->GetShapeType() & (~physics::Entity::SHAPE))
    {
      case physics::Entity::PLANE_SHAPE:
      {
        boost::shared_ptr<physics::PlaneShape> p =
          boost::dynamic_pointer_cast<physics::PlaneShape>((*ci)->GetShape());

        // Add a contact surface to represent the ground.
        // Half space normal is -x; must rotate about y to make it +z.
        this->matter.Ground().updBody().addContactSurface(Rotation(Pi/2, YAxis),
           ContactSurface(ContactGeometry::HalfSpace(), material));

        Vec3 normal = SimbodyPhysics::Vector3ToVec3(p->GetNormal());

        // by default, simbody HalfSpace normal is in the -X direction
        // rotate it based on normal vector specified by user
        // Create a rotation whos x-axis is in the
        // negative normal vector direction
        Rotation R_XN(-UnitVec3(normal), XAxis);

        ContactSurface surface(ContactGeometry::HalfSpace(), material);

        if (addModelClique)
            surface.joinClique(_modelClique);

        _mobod.updBody().addContactSurface(R_XN, surface);
      }
      break;

      case physics::Entity::SPHERE_SHAPE:
      {
        boost::shared_ptr<physics::SphereShape> s =
          boost::dynamic_pointer_cast<physics::SphereShape>((*ci)->GetShape());
        double r = s->GetRadius();
        ContactSurface surface(ContactGeometry::Sphere(r), material);
        if (addModelClique)
            surface.joinClique(_modelClique);
        _mobod.updBody().addContactSurface(X_LC, surface);
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
        const PolygonalMesh mesh = PolygonalMesh::
            createCylinderMesh(ZAxis, r, len/2, resolution);
        const ContactGeometry::TriangleMesh triMesh(mesh);
        ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // Vec3 esz = Vec3(r, r, len/2);  // Use ellipsoid instead
        // ContactSurface surface(ContactGeometry::Ellipsoid(esz),
        //                        material);

        if (addModelClique)
            surface.joinClique(_modelClique);
        _mobod.updBody().addContactSurface(X_LC, surface);
      }
      break;

      case physics::Entity::BOX_SHAPE:
      {
        Vec3 hsz = SimbodyPhysics::Vector3ToVec3(
          (boost::dynamic_pointer_cast<physics::BoxShape>(
          (*ci)->GetShape()))->GetSize())/2;

        /// \TODO: make collision resolution an adjustable parameter
        // number times to chop the longest side.
        const int resolution = 2;
        // const int resolution = 10 * (int)(max(hsz)/min(hsz) + 0.5);
        const PolygonalMesh mesh = PolygonalMesh::
            createBrickMesh(hsz, resolution);
        const ContactGeometry::TriangleMesh triMesh(mesh);
        ContactSurface surface(triMesh, material, 1 /*Thickness*/);

        // ContactSurface surface(ContactGeometry::Ellipsoid(hsz),
        //                        material);

        if (addModelClique)
            surface.joinClique(_modelClique);
        _mobod.updBody().addContactSurface(X_LC, surface);
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
SimTK::Quaternion SimbodyPhysics::QuadToQuad(const math::Quaternion &_q)
{
  return SimTK::Quaternion(_q.w, _q.x, _q.y, _q.z);
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
math::Vector3 SimbodyPhysics::Vec3ToVector3(const SimTK::Vec3 &_v)
{
  return math::Vector3(_v[0], _v[1], _v[2]);
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
math::Pose SimbodyPhysics::Transform2Pose(const SimTK::Transform &_xAB)
{
  SimTK::Quaternion q(_xAB.R());
  const SimTK::Vec4 &qv = q.asVec4();
  return math::Pose(math::Vector3(_xAB.p()[0], _xAB.p()[1], _xAB.p()[2]),
    math::Quaternion(qv[0], qv[1], qv[2], qv[3]));
}

/////////////////////////////////////////////////
SimTK::Transform SimbodyPhysics::GetPose(sdf::ElementPtr _element)
{
  const math::Pose pose = _element->Get<math::Pose>("pose");
  return Pose2Transform(pose);
}

/////////////////////////////////////////////////
std::string SimbodyPhysics::GetTypeString(unsigned int _type)
{
  return GetTypeString(physics::Base::EntityType(_type));
}
