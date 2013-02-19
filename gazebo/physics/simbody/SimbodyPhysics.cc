/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: The Simbody physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#include "physics/simbody/SimbodyTypes.hh"
#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyCollision.hh"

#include "physics/simbody/SimbodyPlaneShape.hh"
#include "physics/simbody/SimbodySphereShape.hh"
#include "physics/simbody/SimbodyHeightmapShape.hh"
#include "physics/simbody/SimbodyMultiRayShape.hh"
#include "physics/simbody/SimbodyBoxShape.hh"
#include "physics/simbody/SimbodyCylinderShape.hh"
#include "physics/simbody/SimbodyTrimeshShape.hh"
#include "physics/simbody/SimbodyRayShape.hh"

#include "physics/simbody/SimbodyHingeJoint.hh"
#include "physics/simbody/SimbodyUniversalJoint.hh"
#include "physics/simbody/SimbodyBallJoint.hh"
#include "physics/simbody/SimbodySliderJoint.hh"
#include "physics/simbody/SimbodyHinge2Joint.hh"
#include "physics/simbody/SimbodyScrewJoint.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Collision.hh"
#include "physics/MapShape.hh"

#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"

#include "SimbodyPhysics.hh"

// include simbody
#include "Simbody.h"

using namespace gazebo;
using namespace physics;
using namespace SimTK;

GZ_REGISTER_PHYSICS_ENGINE("simbody", SimbodyPhysics)

//////////////////////////////////////////////////
bool ContactCallback()
{
  return true;
}

//////////////////////////////////////////////////
bool ContactProcessed()
{
  return true;
}

//////////////////////////////////////////////////
SimbodyPhysics::SimbodyPhysics(WorldPtr _world)
    : PhysicsEngine(_world), system(), matter(system), forces(system),
      tracker(system), contact(system, tracker),  integ(NULL)
{
  // Instantiate the Multibody System
  // Instantiate the Simbody Matter Subsystem
  // Instantiate the Simbody General Force Subsystem

  // Create an integrator
  //this->integ = new SimTK::RungeKuttaMersonIntegrator(system);
  this->integ = new SimTK::ExplicitEulerIntegrator(system);



}

//////////////////////////////////////////////////
SimbodyPhysics::~SimbodyPhysics()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr simbodyElem = this->sdf->GetElement("simbody");

  this->stepTimeDouble = simbodyElem->GetElement("dt")->GetValueDouble();

  math::Vector3 g = this->sdf->GetValueVector3("gravity");


  // set gravity
  SimTK::Force::UniformGravity(this->forces, this->matter,
    SimTK::Vec3(g.x, g.y, g.z));
}

//////////////////////////////////////////////////
void SimbodyPhysics::Init()
{
  try {
    //----------------------- GENERATE MULTIBODY GRAPH -------------------------
    MultibodyGraphMaker mbgraph;
    createMultibodyGraph(mbgraph);
    // Optional: dump the graph to stdout for debugging or curiosity.
    mbgraph.dumpGraph(gzdbg);

    //------------------------ CREATE SIMBODY SYSTEM ---------------------------
    // Create a Simbody System and populate it with Subsystems we'll need.
    SimbbodyPhysics::BuildSimbodySystem(mbgraph, this->GetGravity(),
                       this->system, this->matter, this->forces, this->contact);

  } catch (const std::exception& e) {
      cout << "Simbody build EXCEPTION: " << e.what() << "\n";
      return 1;
  }


  // add pendulum body
  // default pin joint has axis in the +z direction,
  // rotate to get pin in x direction
  const SimTK::Rotation ZtoY( -SimTK::Pi/2, SimTK::XAxis);
  SimTK::Body::Rigid pendulumBody(MassProperties(1.0, Vec3(-1, 0, 0), UnitInertia::pointMassAt(Vec3(-1,0,0))));
  SimTK::MobilizedBody::Pin pendulum(matter.Ground(), Transform(ZtoY, Vec3(0, 0, 0)), 
                                pendulumBody,    Transform(ZtoY, Vec3(1, 0, 0)));

  SimTK::State state = this->system.realizeTopology();

  // pendulum.setAngle(state, Pi/4);

  this->integ->initialize(state);
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
    this->integ->stepTo(this->world->GetSimTime().Double(),
                       this->world->GetSimTime().Double());

  const SimTK::State &s = integ->getState();
  gzerr << "time [" << s.getTime()
        << "] q [" << s.getQ()
        << "] u [" << s.getU()
        << "] dt [" << this->stepTimeDouble
        << "] t [" << this->world->GetSimTime().Double()
        << "]\n";

  ModelPtr model = this->world->GetModel("model1");
  LinkPtr link = model->GetLink("link1");

  JointPtr joint = model->GetJoint("model1::joint1");
  joint->SetAngle(0, s.getQ()[0]);

  link->SetWorldPose(math::Pose(
    math::Vector3(0, 0, s.getQ()[0]),
    math::Quaternion(0, 0, 0)));
  // set link pose
  this->world->dirtyPoses.push_back(
    boost::shared_static_cast<physics::Entity>(link).get());

  this->lastUpdateTime = currTime;
}

//////////////////////////////////////////////////
void SimbodyPhysics::Fini()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetStepTime(double _value)
{
  this->sdf->GetElement("simbody")->GetElement(
      "solver")->GetAttribute("min_step_size")->Set(_value);

  this->stepTimeDouble = _value;
}

//////////////////////////////////////////////////
double SimbodyPhysics::GetStepTime()
{
  return this->stepTimeDouble;
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
    boost::shared_dynamic_cast<SimbodyCollision>(_collision);

  if (_type == "plane")
    shape.reset(new SimbodyPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new SimbodySphereShape(collision));
  else if (_type == "box")
    shape.reset(new SimbodyBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new SimbodyCylinderShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new SimbodyTrimeshShape(collision));
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

  /*
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
    */
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
void SimbodyPhysics::ConvertMass(InertialPtr /*_inertial*/,
                                void * /*_engineMass*/)
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::ConvertMass(void * /*_engineMass*/,
                                const InertialPtr /*_inertial*/)
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->GetAttribute("xyz")->Set(_gravity);
}

//////////////////////////////////////////////////
void SimbodyPhysics::DebugPrint() const
{
}

//////////////////////////////////////////////////
//==============================================================================
//                           CREATE MULTIBODY GRAPH
//==============================================================================
// Define Gazebo joint types, then use links and joints in the given model
// to construct a reasonable spanning-tree-plus-constraints multibody graph
// to represent that model. An exception will be thrown if this fails.
// Note that this step is not Simbody dependent.
void SimbodyPhysics::CreateMultibodyGraph(MultibodyGraphMaker&   mbgraph)
{
  // Step 1: Tell MultibodyGraphMaker about joints it should know about.
  // Note: "weld" and "free" are always predefined at 0 and 6 dofs, resp.
  //                  Gazebo name  #dofs     Simbody equivalent
  mbgraph.addJointType(GetTypeString(physics::Base::HINGE_JOINT),  1);
  mbgraph.addJointType(GetTypeString(physics::Base::HINGE2_JOINT), 2);
  mbgraph.addJointType(GetTypeString(physics::Base::SLIDER_JOINT), 1);
  mbgraph.addJointType(GetTypeString(physics::Base::UNIVERSAL_JOINT), 2);
  mbgraph.addJointType(GetTypeString(physics::Base::SCREW_JOINT), 1);
  // Simbody has a Ball constraint that is a good choice if you need to
  // break a loop at a ball joint.
  mbgraph.addJointType(GetTypeString(physics::Base::BALL_JOINT), 3, true);

  // Step 2: Tell it about all the links we read from the input file, 
  // starting with world, and provide a reference pointer.
  physics::Model_V models = this->world->GetModels();
  for (physics::Model_V::iterator mi = models.begin();
       mi != models.end(); ++mi)
  {
    physics::Link_V links = mi->GetLinks();
    for (physics::Link_V::iterator li = links.begin();
         li != links.end(); ++li)
    {
      SimbodyLinkPtr simbodyLink = boost::share_dynamic_cast<SimbodyLink>(*li);
      if (simbodyLink)
        mbgraph.addBody(li->GetName(), li->GetInertial()->GetMass(),
                        simbodyLink->mustBeBaseLink, simbodyLink->get());
      else
        gzerr << "simbodyLink [" << li->GetName() << "]is not a SimbodyLinkPtr\n";
    }

    // Step 3: Tell it about all the joints we read from the input file,
    // and provide a reference pointer.
    physics::Joint_V joints = mi->GetJoints();
    for (physics::Joint_V::iterator ji = Joints.begin();
         ji != Joints.end(); ++ji)
    {
      SimbodyJointPtr simbodyJoint = boost::share_dynamic_cast<SimbodyJoint>(*ji);
      if (simbodyJoint)
        mbgraph.addJoint(ji->GetName(), GetTypeString(ji->GetType()),
           ji->GetParent()->GetName(), ji->GetChild()->GetName(),
                            simbodyJoint->mustBreakLoopHere, simbodyJoint->get());
      else
        gzerr << "simbodyJoint [" << ji->GetName() << "]is not a SimbodyJointPtr\n";
    }
  }

  // Setp 4. Generate the multibody graph.
  mbgraph.generateGraph();
}

//////////////////////////////////////////////////
//==============================================================================
//                            BUILD SIMBODY SYSTEM
//==============================================================================
// Given a desired multibody graph, gravity, and the Gazebo model that was
// used to generate the graph, create a Simbody System for it. There are many
// limitations here, especially in the handling of contact. Any Gazebo features
// that we haven't modeled are just ignored.
// The GazeboModel is updated so that its links and joints have references to
// their corresponding Simbody elements.
// We set up some visualization here so we can see what's happening but this
// would not be needed in Gazebo since it does its own visualization.
void SimbodyPhysics::BuildSimbodySystem(const SimTK::MultibodyGraphMaker& mbgraph,
                               const math::Vector3&       gravity,
                               SimTK::MultibodySystem&           mbs,
                               SimTK::SimbodyMatterSubsystem&    matter,
                               SimTK::GeneralForceSubsystem&     forces,
                               SimTK::CompliantContactSubsystem& contact) 
{
    const SimTK::Vec3 g(gravity.x, gravity.y, gravity.z);

    // Set stiction max slip velocity to make it less stiff.
    contact.setTransitionVelocity(0.1);

    // Specify gravity (read in above from world).
    SimTK::Force::UniformGravity(forces, matter, g);

    // TODO: Edit physics::Surface class to support these properties
    // Define a material to use for contact. This is not very stiff.
    SimTK::ContactMaterial material(1e6,   // stiffness
                                     0.1,  // dissipation
                                     0.7,   // mu_static
                                     0.5,   // mu_dynamic
                                     0.5);  // mu_viscous

    // Add a contact surface to represent the ground.
    // Half space normal is -x; must rotate about y to make it +z.
    matter.Ground().updBody().addContactSurface(Rotation(Pi/2,YAxis),
       ContactSurface(ContactGeometry::HalfSpace(), material));

    // Generate a contact clique we can put collision geometry in to prevent
    // self-collisions.
    ContactCliqueId modelClique = ContactSurface::createNewContactClique();

    // Will specify explicitly when needed
    // Record the MobilizedBody for the World link.
    // model.links.updLink(0).masterMobod = matter.Ground();

    // Run through all the mobilizers in the multibody graph, adding a Simbody
    // MobilizedBody for each one. Also add visual and collision geometry to the
    // bodies when they are mobilized.
    for (int mobNum=0; mobNum < mbgraph.getNumMobilizers(); ++mobNum) {
        // Get a mobilizer from the graph, then extract its corresponding
        // joint and bodies. Note that these don't necessarily have equivalents
        // in the GazeboLink and GazeboJoint inputs.
        const MultibodyGraphMaker::Mobilizer& mob = mbgraph.getMobilizer(mobNum);
        const std::string& type = mob.getJointTypeName();

        // The inboard body always corresponds to one of the input links,
        // because a slave link is always the outboard body of a mobilizer.
        // The outboard body may be slave, but its master body is one of the
        // Gazebo input links.
        const bool isSlave = mob.isSlaveMobilizer();
        GazeboLinkInfo& gzInb  = *(GazeboLinkInfo*)mob.getInboardBodyRef();
        GazeboLinkInfo& gzOutb = *(GazeboLinkInfo*)mob.getOutboardMasterBodyRef();

        const MassProperties massProps = 
            gzOutb.getEffectiveMassProps(mob.getNumFragments());

        // This will reference the new mobilized body once we create it.
        MobilizedBody mobod; 

        if (mob.isAddedBaseMobilizer()) {
            // There is no corresponding Gazebo joint for this mobilizer.
            // Create the joint and set its default position to be the default
            // pose of the base link relative to the Ground frame.
            assert(type=="free"); // May add more types later
            if (type == "free") {
                MobilizedBody::Free freeJoint(
                    gzInb.masterMobod,  Transform(),
                    massProps,    Transform());
                freeJoint.setDefaultTransform(~gzInb.defX_ML*gzOutb.defX_ML);
                mobod = freeJoint;
            }
        } else {
            // This mobilizer does correspond to one of the input joints.
            GazeboJointInfo& gzJoint = *(GazeboJointInfo*)mob.getJointRef();
            const bool isReversed = mob.isReversedFromJoint();

            // Find inboard and outboard frames for the mobilizer; these are
            // parent and child frames or the reverse.

            const Transform& X_IF0 = isReversed ? gzJoint.X_CB : gzJoint.X_PA;
            const Transform& X_OM0 = isReversed ? gzJoint.X_PA : gzJoint.X_CB;

            const MobilizedBody::Direction direction =
                isReversed ? MobilizedBody::Reverse : MobilizedBody::Forward;

            if (type == "free") {
                MobilizedBody::Free freeJoint(
                    gzInb.masterMobod,  X_IF0,
                    massProps,          X_OM0, 
                    direction);
                Transform defX_FM = isReversed ? Transform(~gzJoint.defX_AB)
                                               : gzJoint.defX_AB;
                freeJoint.setDefaultTransform(defX_FM);
                mobod = freeJoint;
            } else if (type == "revolute") {
                Xml::Element axisElt = gzJoint.element.getRequiredElement("axis");
                UnitVec3 axis = 
                    UnitVec3(axisElt.getRequiredElementValueAs<Vec3>("xyz")); 
                Rotation R_JZ(axis, ZAxis); // Simbody's pin is along Z
                Transform X_IF(X_IF0.R()*R_JZ, X_IF0.p());
                Transform X_OM(X_OM0.R()*R_JZ, X_OM0.p());
                MobilizedBody::Pin pinJoint(
                    gzInb.masterMobod,      X_IF,
                    massProps,              X_OM, 
                    direction);
                mobod = pinJoint;

                #ifdef ADD_JOINT_SPRINGS
                // KLUDGE add spring (stiffness proportional to mass)
                Force::MobilityLinearSpring(forces,mobod,0,
                                            30*massProps.getMass(),0);
                #endif
            } else if (type == "prismatic") {
                Xml::Element axisElt = gzJoint.element.getRequiredElement("axis");
                UnitVec3 axis = 
                    UnitVec3(axisElt.getRequiredElementValueAs<Vec3>("xyz")); 
                Rotation R_JX(axis, XAxis); // Simbody's slider is along X
                Transform X_IF(X_IF0.R()*R_JX, X_IF0.p());
                Transform X_OM(X_OM0.R()*R_JX, X_OM0.p());
                MobilizedBody::Slider sliderJoint(
                    gzInb.masterMobod,      X_IF,
                    massProps,              X_OM, 
                    direction);
                mobod = sliderJoint;

                #ifdef ADD_JOINT_SPRINGS
                // KLUDGE add spring (stiffness proportional to mass)
                Force::MobilityLinearSpring(forces,mobod,0,
                                            30*massProps.getMass(),0);
                #endif
            } else if (type == "ball") {
                MobilizedBody::Ball ballJoint(
                    gzInb.masterMobod,  X_IF0,
                    massProps,          X_OM0, 
                    direction);
                Rotation defR_FM = isReversed 
                    ? Rotation(~gzJoint.defX_AB.R())
                    : gzJoint.defX_AB.R();
                ballJoint.setDefaultRotation(defR_FM);
                mobod = ballJoint;
            } 

            // Created a mobilizer that corresponds to gzJoint. Keep track.
            gzJoint.mobod = mobod;
            gzJoint.isReversed = isReversed;
        }

        // Link gzOutb has been mobilized; keep track for later.
        if (isSlave) gzOutb.slaveMobods.push_back(mobod);
        else gzOutb.masterMobod = mobod;

        // A mobilizer has been created; now add the visual and collision
        // geometry for the new mobilized body.

        Xml::Element master = gzOutb.element;
        Vec3 color = isSlave ? Red : Cyan;
        Real scale = isSlave ? 0.9 : 1.;
        if (master.isValid()) {
            // VISUAL
            Array_<Xml::Element> visuals = master.getAllElements("visual");
            for (unsigned i=0; i < visuals.size(); ++i)  {
                Transform X_LV = getPose(visuals[i]);
                Xml::Element geo = visuals[i].getRequiredElement("geometry");
                Xml::Element box = geo.getOptionalElement("box");
                if (box.isValid()) {
                    Vec3 sz = box.getRequiredElementValueAs<Vec3>("size");
                    mobod.addBodyDecoration(
                        X_LV, DecorativeBrick(sz/2).setOpacity(.5)
                                .setColor(color).setScale(scale));
                }
                Xml::Element sphere = geo.getOptionalElement("sphere");
                if (sphere.isValid()) {
                    Real r = sphere.getRequiredElementValueAs<Real>("radius");
                    mobod.addBodyDecoration(
                        X_LV, DecorativeSphere(r).setOpacity(.5)
                                .setColor(color).setScale(scale));
                }
                Xml::Element cyl = geo.getOptionalElement("cylinder");
                if (cyl.isValid()) {
                    Real r = cyl.getRequiredElementValueAs<Real>("radius");
                    Real l = cyl.getRequiredElementValueAs<Real>("length");
                    // Cylinder is along Z in Gazebo, Y in Simbody
                    Rotation YtoZ(Pi/2, XAxis);
                    mobod.addBodyDecoration(
                        Transform(X_LV.R()*YtoZ, X_LV.p()),
                        DecorativeCylinder(r, l/2).setOpacity(.5)
                                .setColor(color).setScale(scale));
                }

            } 

            // COLLISION
            Array_<Xml::Element> coll = master.getAllElements("collision");
            for (unsigned i=0; i < coll.size(); ++i) {
                Transform X_LC = getPose(coll[i]);
                Xml::Element geo = coll[i].getRequiredElement("geometry");

                // Model sphere collision surface.
                Xml::Element sphere = geo.getOptionalElement("sphere");
                if (sphere.isValid()) {
                    Real r = sphere.getRequiredElementValueAs<Real>("radius");
                    mobod.addBodyDecoration(
                        X_LC, DecorativeSphere(r)
                                .setRepresentation(DecorativeGeometry::DrawWireframe)
                                .setColor(Gray));
                    ContactSurface surface(ContactGeometry::Sphere(r),
                                           material);
                    if (!gzOutb.selfCollide)
                        surface.joinClique(model.modelClique);
                    mobod.updBody().addContactSurface(X_LC, surface);
                }

                // Model cylinder collision surface (must fake with ellipsoid).
                Xml::Element cyl = geo.getOptionalElement("cylinder");
                if (cyl.isValid()) {
                    Real r   = cyl.getRequiredElementValueAs<Real>("radius");
                    Real len = cyl.getRequiredElementValueAs<Real>("length");
                    // Cylinder is along Z in Gazebo
                    Vec3 esz = Vec3(r,r,len/2); // Use ellipsoid instead
                    mobod.addBodyDecoration(X_LC, 
                        DecorativeEllipsoid(esz)
                            .setRepresentation(DecorativeGeometry::DrawWireframe)
                            .setColor(Gray));
                    ContactSurface surface(ContactGeometry::Ellipsoid(esz),
                                           material);
                    if (!gzOutb.selfCollide)
                        surface.joinClique(model.modelClique);
                    mobod.updBody().addContactSurface(X_LC, surface);
                }


                // Model box collision surface (must fake with ellipsoid).
                Xml::Element box = geo.getOptionalElement("box");
                if (box.isValid()) {
                    Vec3 hsz = box.getRequiredElementValueAs<Vec3>("size")/2;
                    mobod.addBodyDecoration(X_LC, 
                        DecorativeEllipsoid(hsz) // use half dimensions
                            .setRepresentation(DecorativeGeometry::DrawWireframe)
                            .setColor(Gray));
                    ContactSurface surface(ContactGeometry::Ellipsoid(hsz),
                                           material);
                    if (!gzOutb.selfCollide)
                        surface.joinClique(model.modelClique);
                    mobod.updBody().addContactSurface(X_LC, surface);
                }
            }
        }
    }

    // Weld the slaves to their masters.
    for (int lx=0; lx < model.links.size(); ++lx) {
        GazeboLinkInfo& link = model.links.updLink(lx);
        if (link.slaveMobods.empty()) continue;
        for (unsigned i=0; i < link.slaveMobods.size(); ++i) {
            Constraint::Weld weld(link.masterMobod, link.slaveMobods[i]);
            link.slaveWelds.push_back(weld); // in case we want to know later
        }
    }

    // Add the loop joints if any.
    for (int lcx=0; lcx < mbgraph.getNumLoopConstraints(); ++lcx) {
        const MultibodyGraphMaker::LoopConstraint& loop =
            mbgraph.getLoopConstraint(lcx);

        GazeboJointInfo& joint  = *(GazeboJointInfo*)loop.getJointRef();
        GazeboLinkInfo&  parent = *(GazeboLinkInfo*) loop.getParentBodyRef();
        GazeboLinkInfo&  child  = *(GazeboLinkInfo*) loop.getChildBodyRef();

        if (joint.type == "weld") {
            Constraint::Weld weld(parent.masterMobod, joint.X_PA, 
                                  child.masterMobod,  joint.X_CB);
            joint.constraint = weld;
        } else if (joint.type == "ball") {
            Constraint::Ball ball(parent.masterMobod, joint.X_PA.p(), 
                                  child.masterMobod,  joint.X_CB.p());
            joint.constraint = ball;
        } else if (joint.type == "free") {
            // A "free" loop constraint is no constraint at all so we can
            // just ignore it. It might be more convenient if there were
            // a 0-constraint Constraint::Free, just as there is a 0-mobility
            // MobilizedBody::Weld.
        } else
            throw std::runtime_error(
                "Unrecognized loop constraint type '" + joint.type + "'.");
    }
}



