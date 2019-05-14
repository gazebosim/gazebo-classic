/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/World.hh"

#include "gazebo/physics/dart/DARTScrewJoint.hh"
#include "gazebo/physics/dart/DARTHingeJoint.hh"
#include "gazebo/physics/dart/DARTHinge2Joint.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"
#include "gazebo/physics/dart/DARTBallJoint.hh"
#include "gazebo/physics/dart/DARTUniversalJoint.hh"
#include "gazebo/physics/dart/DARTFixedJoint.hh"

#include "gazebo/physics/dart/DARTRayShape.hh"
#include "gazebo/physics/dart/DARTBoxShape.hh"
#include "gazebo/physics/dart/DARTSphereShape.hh"
#include "gazebo/physics/dart/DARTCylinderShape.hh"
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"
#include "gazebo/physics/dart/DARTPolylineShape.hh"
#include "gazebo/physics/dart/DARTMultiRayShape.hh"
#include "gazebo/physics/dart/DARTHeightmapShape.hh"

#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"

#include "gazebo/physics/dart/DARTPhysics.hh"

#include "gazebo/physics/dart/DARTPhysicsPrivate.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("dart", DARTPhysics)

//////////////////////////////////////////////////
DARTPhysics::DARTPhysics(WorldPtr _world)
    : PhysicsEngine(_world), dataPtr(new DARTPhysicsPrivate())
{
}

//////////////////////////////////////////////////
DARTPhysics::~DARTPhysics()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Gravity
  auto g = this->world->Gravity();
  // ODEPhysics checks this, so we will too.
  if (g == ignition::math::Vector3d::Zero)
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->dataPtr->dtWorld->setGravity(Eigen::Vector3d(g.X(), g.Y(), g.Z()));
}

//////////////////////////////////////////////////
void DARTPhysics::Init()
{
}

//////////////////////////////////////////////////
void DARTPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void DARTPhysics::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // Restore state all the models
  unsigned int modelCount = this->world->ModelCount();
  DARTModelPtr dartModelIt;

  for (unsigned int i = 0; i < modelCount; ++i)
  {
    dartModelIt =
      boost::dynamic_pointer_cast<DARTModel>(this->world->ModelByIndex(i));
    GZ_ASSERT(dartModelIt, "dartModelIt pointer is null");

    dartModelIt->RestoreState();
  }
}

//////////////////////////////////////////////////
void DARTPhysics::InitForThread()
{
}


//////////////////////////////////////////////////
// Helper class to maintain unique pairs of links in maps
class LinkPair
{
  /// \brief Constructor.
  /// \param _link1[in] and _link2[in] are the DART links for the contact.
  ///   Their order may be swapped around.
  public: LinkPair(const DARTLinkPtr &_link1,
                   const DARTLinkPtr &_link2)
          {
            this->SetDARTLinks(_link1, _link2);
          }

  /// \brief Copy constructor
  public: LinkPair(const LinkPair&_linkPair):
            dtLink1(_linkPair.dtLink1),
            dtLink2(_linkPair.dtLink2)
          {}

  /// \brief Destructor.
  public: virtual ~LinkPair() {}

  /// \brief Comparison operator
  /// which always chooses the link which evaluates to lower by operator <
  /// \param[in] _dtLink1 the first link
  /// \param[in] _dtLink2 the second link
  public: void SetDARTLinks(const DARTLinkPtr &_dtLink1,
                            const DARTLinkPtr &_dtLink2)
          {
            if (cmpLink(_dtLink1, _dtLink2) < 0)
            {
              this->dtLink1 = _dtLink1;
              this->dtLink2 = _dtLink2;
            }
            else
            {
              this->dtLink2 = _dtLink1;
              this->dtLink1 = _dtLink2;
            }
          }

  /// \brief Comparison operator
  /// \param[in] c the link pair
  /// \return true if this is lower than \e c
  public: bool operator<(const LinkPair& c) const
          {
            int cmp1 = cmpLink(this->dtLink1, c.dtLink1);
            if (cmp1 < 0) return true;
            int cmp2 = cmpLink(this->dtLink2, c.dtLink2);
            return (cmp1 == 0) && (cmp2 < 0);
          }

  // \brief returns the first link
  public: DARTLinkPtr DARTLink1() const { return this->dtLink1; }

  // \brief returns the second link
  public: DARTLinkPtr DARTLink2() const { return this->dtLink2; }

  // \brief internally used comparison operator for links
  // \param[in] l1 the first link
  // \param[in] l2 the second link
  // \retval -1 l1 < l2
  // \retval 0 l1 == l2
  // \retval 1 l1 > l2
  private: int cmpLink(const DARTLinkPtr &l1, const DARTLinkPtr &l2) const
           {
            // Comparing by address is required because names can be
            // the same for different links. Could use the model name of the
            // link in addition, but that's even more expensive.
             return l1.get() < l2.get() ? -1 : l1.get() == l2.get() ? 0 : 1;
           }

  /// \brief Pointer to the first link object
  private: DARTLinkPtr dtLink1;

  /// \brief Pointer to the second link object
  private: DARTLinkPtr dtLink2;
};

//////////////////////////////////////////////////
static DARTLinkPtr StaticFindDARTLink(
    DARTPhysics *_dtPhysics,
    const dart::dynamics::BodyNode *_dtBodyNode)
{
  DARTLinkPtr res;

  const Model_V& models = _dtPhysics->World()->Models();

  for (Model_V::const_iterator itModel = models.begin();
       itModel != models.end(); ++itModel)
  {
    const Link_V& links = (*itModel)->GetLinks();

    for (Link_V::const_iterator itLink = links.begin();
         itLink != links.end(); ++itLink)
    {
      DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(*itLink);

      if (dartLink->DARTBodyNode() == _dtBodyNode)
      {
        res = dartLink;
        break;
      }
    }
  }

  return res;
}

//////////////////////////////////////////////////
static void RetrieveDARTCollisions(
    DARTPhysics* _dtPhysics,
    const dart::collision::CollisionResult *_dtLastResult,
    ContactManager *_mgr)
{
  _mgr->ResetCount();
  int numContacts = _dtLastResult->getNumContacts();

  // DART returns all contact points individually, without grouping
  // them to link pairs first. The majority of the Gazebo code assumes
  // the contacts will come per link pair (e.g. all contacts of
  // link1 and link2 grouped together in one Contact object).
  // We will have to do this mapping here first.
  // This could be skipped for performance reasons by supplying
  // an additional std::vector<std::pair<LinkPair, dart::collision::Contact>
  // which is just filled with duplicates, skipping the map step.

  typedef std::map<LinkPair, std::deque<const dart::collision::Contact*>>
    PairedContactsMap;
  // Attention: The map uses a reference to dart::collision::Contact&, but this
  // is retrieved from dtLastResult directly and used only within this function,
  // so it will be safe to use within the scope of this function.
  PairedContactsMap pairedContacts;

  // insert all the contacts
  for (int i = 0; i < numContacts; ++i)
  {
    const dart::collision::Contact &dtContact =
        _dtLastResult->getContact(i);

    dart::collision::CollisionObject *dtCollObj1 = dtContact.collisionObject1;
    dart::collision::CollisionObject *dtCollObj2 = dtContact.collisionObject2;

    GZ_ASSERT(dtCollObj1, "collision object 1 is null!");
    GZ_ASSERT(dtCollObj2, "collision object 2 is null!");

    dart::dynamics::ConstBodyNodePtr dtBodyNode1;
    dart::dynamics::ConstBodyNodePtr dtBodyNode2;

    const dart::dynamics::ShapeFrame *dtShapeFrame1 =
      dtCollObj1->getShapeFrame();
    const dart::dynamics::ShapeFrame *dtShapeFrame2 =
      dtCollObj2->getShapeFrame();

    GZ_ASSERT(dtShapeFrame1, "shape frame 1 is null!");
    GZ_ASSERT(dtShapeFrame2, "shape frame 2 is null!");
    GZ_ASSERT(dtShapeFrame1->asShapeNode(), "shape frame 1 is no shape node!");
    GZ_ASSERT(dtShapeFrame2->asShapeNode(), "shape frame 2 is no shape node!");

    if (dtShapeFrame1->isShapeNode())
      dtBodyNode1 = dtShapeFrame1->asShapeNode()->getBodyNodePtr();
    if (dtShapeFrame2->isShapeNode())
      dtBodyNode2 = dtShapeFrame2->asShapeNode()->getBodyNodePtr();

    GZ_ASSERT(dtBodyNode1, "body node 1 is null!");
    GZ_ASSERT(dtBodyNode2, "body node 2 is null!");

    DARTLinkPtr dartLink1 = StaticFindDARTLink(_dtPhysics, dtBodyNode1);
    DARTLinkPtr dartLink2 = StaticFindDARTLink(_dtPhysics, dtBodyNode2);

    GZ_ASSERT(dartLink1, "dartLink1 in collision pair is null");
    GZ_ASSERT(dartLink2, "dartLink2 in collision pair is null");

    LinkPair dtLinkPair(dartLink1, dartLink2);
    pairedContacts[dtLinkPair].push_back(&dtContact);
  }

  for (PairedContactsMap::iterator it = pairedContacts.begin();
       it != pairedContacts.end(); ++it)
  {
    DARTLinkPtr dartLink1 = it->first.DARTLink1();
    DARTLinkPtr dartLink2 = it->first.DARTLink2();
    const std::deque<const dart::collision::Contact*> &dtContacts = it->second;

    GZ_ASSERT(!dtContacts.empty(),
           "dtContacts is empty, at least one contact should have been added");

    unsigned int colIndex = 0;
    CollisionPtr collisionPtr1 = dartLink1->GetCollision(colIndex);
    CollisionPtr collisionPtr2 = dartLink2->GetCollision(colIndex);

    // Add a new contact to the manager. This will return nullptr if no one is
    // listening for contact information.
    // It would be nice to do this in the first loop in order to save
    // computation, but we can only add a new contact once per link pair,
    // which is information we only have after the first loop.
    // We could avoid all the computation however if the contact manager
    // had a function returning in advance whether the ContactManger::NewContact
    // will return NULL!
    Contact *contactFeedback = _mgr->NewContact(
                                 collisionPtr1.get(), collisionPtr2.get(),
                                 _dtPhysics->World()->SimTime());
    if (!contactFeedback)
      continue;

    auto body1Pose = dartLink1->WorldPose();
    auto body2Pose = dartLink2->WorldPose();
    dart::dynamics::BodyNode *dtBodyNode1 = dartLink1->DARTBodyNode();
    dart::dynamics::BodyNode *dtBodyNode2 = dartLink2->DARTBodyNode();

    contactFeedback->count = 0;

    std::deque<const dart::collision::Contact*>::const_iterator contIt;
    int contNum = 0;
    for (contIt = dtContacts.begin();
         (contNum < MAX_CONTACT_JOINTS) && (contIt != dtContacts.end());
         ++contIt, ++contNum)
    {
      const dart::collision::Contact *dtContact = *contIt;

      ignition::math::Vector3d localForce1;
      ignition::math::Vector3d localForce2;
      ignition::math::Vector3d localTorque1;
      ignition::math::Vector3d localTorque2;

      // calculate force in world frame
      Eigen::Vector3d force = dtContact->force;

      // calculate torque in world frame
      Eigen::Vector3d torqueA =
          (dtContact->point -
           dtBodyNode1->getTransform().translation()).cross(force);
      Eigen::Vector3d torqueB =
          (dtContact->point -
           dtBodyNode2->getTransform().translation()).cross(-force);

      // Convert from world to link frame
      localForce1 = body1Pose.Rot().RotateVectorReverse(
          DARTTypes::ConvVec3Ign(force));
      localForce2 = body2Pose.Rot().RotateVectorReverse(
          DARTTypes::ConvVec3Ign(-force));
      localTorque1 = body1Pose.Rot().RotateVectorReverse(
          DARTTypes::ConvVec3Ign(torqueA));
      localTorque2 = body2Pose.Rot().RotateVectorReverse(
          DARTTypes::ConvVec3Ign(torqueB));

      contactFeedback->positions[contNum] =
        DARTTypes::ConvVec3Ign(dtContact->point);
      contactFeedback->normals[contNum] =
        DARTTypes::ConvVec3Ign(dtContact->normal);
      contactFeedback->depths[contNum] =
        dtContact->penetrationDepth;

      if (!dartLink1->IsStatic())
      {
        contactFeedback->wrench[contNum].body1Force = localForce1;
        contactFeedback->wrench[contNum].body1Torque = localTorque1;
      }

      if (!dartLink2->IsStatic())
      {
        contactFeedback->wrench[contNum].body2Force = localForce2;
        contactFeedback->wrench[contNum].body2Torque = localTorque2;
      }

      ++contactFeedback->count;
    }
  }
}

//////////////////////////////////////////////////
void DARTPhysics::UpdateCollision()
{
  if (!this->world->PhysicsEnabled())
  {
    dart::collision::CollisionResult localResult;

    // collision computation is disabled when UpdatePhysics() is not
    // being called, so do collision detection separately here.
    std::size_t maxContacts = 1000u;
    dart::collision::CollisionOption opt(true, maxContacts);
    // call of checkCollision will not update the result which
    // can be retrieved with
    // this->dataPtr->dtWorld->getLastCollisionResult()
    // so get the results and store them locally.
    this->dataPtr->dtWorld->checkCollision(opt, &localResult);

    RetrieveDARTCollisions(this, &localResult, this->GetContactManager());
  }
}

//////////////////////////////////////////////////
void DARTPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // common::Time currTime =  this->world->GetRealTime();

  this->dataPtr->dtWorld->setTimeStep(this->maxStepSize);
  this->dataPtr->dtWorld->step(
        this->dataPtr->resetAllForcesAfterSimulationStep);

  // Update all the transformation of DART's links to gazebo's links
  // TODO: How to visit all the links in the world?
  unsigned int modelCount = this->world->ModelCount();
  ModelPtr modelItr;

  for (unsigned int i = 0; i < modelCount; ++i)
  {
    modelItr = this->world->ModelByIndex(i);
    // TODO: need to improve speed
    Link_V links = modelItr->GetLinks();
    unsigned int linkCount = links.size();
    DARTLinkPtr dartLinkItr;

    for (unsigned int j = 0; j < linkCount; ++j)
    {
      dartLinkItr
          = boost::dynamic_pointer_cast<DARTLink>(links.at(j));
      dartLinkItr->updateDirtyPoseFromDARTTransformation();
    }
  }

  RetrieveDARTCollisions(
        this,
        &(this->dataPtr->dtWorld->getLastCollisionResult()),
        this->GetContactManager());
}

//////////////////////////////////////////////////
std::string DARTPhysics::GetType() const
{
  return "dart";
}

//////////////////////////////////////////////////
void DARTPhysics::SetSeed(uint32_t /*_seed*/)
{
  gzwarn << "Not implemented yet in DART.\n";
}

//////////////////////////////////////////////////
ModelPtr DARTPhysics::CreateModel(BasePtr _parent)
{
  DARTModelPtr model(new DARTModel(_parent));

  return model;
}

//////////////////////////////////////////////////
LinkPtr DARTPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == nullptr)
  {
    gzerr << "Link must have a parent in DART.\n";
    return LinkPtr();
  }

  DARTLinkPtr link(new DARTLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr DARTPhysics::CreateCollision(const std::string &_type,
                                          LinkPtr _body)
{
  DARTCollisionPtr collision(new DARTCollision(_body));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_body->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr DARTPhysics::CreateShape(const std::string &_type,
                                  CollisionPtr _collision)
{
  DARTCollisionPtr collision =
    boost::dynamic_pointer_cast<DARTCollision>(_collision);

  ShapePtr shape;
  if (_type == "sphere")
    shape.reset(new DARTSphereShape(collision));
  else if (_type == "plane")
    shape.reset(new DARTPlaneShape(collision));
  else if (_type == "box")
    shape.reset(new DARTBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new DARTCylinderShape(collision));
  else if (_type == "multiray")
    shape.reset(new DARTMultiRayShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new DARTMeshShape(collision));
  else if (_type == "polyline")
    shape.reset(new DARTPolylineShape(collision));
  else if (_type == "heightmap")
    shape.reset(new DARTHeightmapShape(collision));
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new DARTRayShape(collision));
    else
      shape.reset(new DARTRayShape(this->world->Physics()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

//////////////////////////////////////////////////
JointPtr DARTPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "prismatic")
    joint.reset(new DARTSliderJoint(_parent));
  else if (_type == "screw")
    joint.reset(new DARTScrewJoint(_parent));
  else if (_type == "revolute")
    joint.reset(new DARTHingeJoint(_parent));
  else if (_type == "revolute2")
    joint.reset(new DARTHinge2Joint(_parent));
  else if (_type == "ball")
    joint.reset(new DARTBallJoint(_parent));
  else if (_type == "universal")
    joint.reset(new DARTUniversalJoint(_parent));
  else if (_type == "fixed")
    joint.reset(new DARTFixedJoint(_parent));
  else
    gzerr << "Unable to create joint of type[" << _type << "]";

  return joint;
}

//////////////////////////////////////////////////
std::string DARTPhysics::GetSolverType() const
{
  if (this->sdf->HasElement("dart"))
  {
    sdf::ElementPtr dartElem = this->sdf->GetElement("dart");
    if (dartElem->HasElement("solver") &&
        dartElem->GetElement("solver")->HasElement("solver_type"))
    {
      return dartElem->GetElement("solver")->Get<std::string>("solver_type");
    }
  }
  return "dantzig";
}

//////////////////////////////////////////////////
void DARTPhysics::SetSolverType(const std::string &_type)
{
  if (_type == "dantzig")
  {
    // DART constraint solver refactored in 6.7, see issue 2605
    // https://bitbucket.org/osrf/gazebo/issues/2605
#if DART_MAJOR_MINOR_VERSION_AT_MOST(6, 6)
    this->dataPtr->dtWorld->getConstraintSolver()->setLCPSolver(
        dart::common::make_unique<dart::constraint::DantzigLCPSolver>(
        this->dataPtr->dtWorld->getTimeStep()));
#else
    auto boxedLCPSolver =
        dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
        this->dataPtr->dtWorld->getConstraintSolver());
    if (boxedLCPSolver)
    {
      boxedLCPSolver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    }
#endif
  }
  else if (_type == "pgs")
  {
    // DART constraint solver refactored in 6.7, see issue 2605
    // https://bitbucket.org/osrf/gazebo/issues/2605
#if DART_MAJOR_MINOR_VERSION_AT_MOST(6, 6)
    this->dataPtr->dtWorld->getConstraintSolver()->setLCPSolver(
        dart::common::make_unique<dart::constraint::PGSLCPSolver>(
        this->dataPtr->dtWorld->getTimeStep()));
#else
    auto boxedLCPSolver =
        dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
        this->dataPtr->dtWorld->getConstraintSolver());
    if (boxedLCPSolver)
    {
      boxedLCPSolver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
    }
#endif
  }
  else
  {
    gzerr << "Invalid step type[" << _type << "]\n";
    return;
  }

  if (this->sdf->HasElement("dart"))
  {
    sdf::ElementPtr dartElem = this->sdf->GetElement("dart");
    if (dartElem->HasElement("solver") &&
        dartElem->GetElement("solver")->HasElement("solver_type"))
    {
      dartElem->GetElement("solver")->GetElement("solver_type")->Set(_type);
    }
  }
}

//////////////////////////////////////////////////
void DARTPhysics::SetGravity(const ignition::math::Vector3d &_gravity)
{
  this->world->SetGravitySDF(_gravity);
  this->dataPtr->dtWorld->setGravity(
    Eigen::Vector3d(_gravity.X(), _gravity.Y(), _gravity.Z()));
}

//////////////////////////////////////////////////
void DARTPhysics::DebugPrint() const
{
  gzwarn << "Not implemented in DART.\n";
}

//////////////////////////////////////////////////
boost::any DARTPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool DARTPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  if (!this->sdf->HasElement("dart"))
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  sdf::ElementPtr dartElem = this->sdf->GetElement("dart");

  // physics dart element not yet added to sdformat
  GZ_ASSERT(dartElem, "DART SDF element does not exist");

  if (_key == "solver_type")
  {
    _value = this->GetSolverType();
  }
  else if (_key == "max_contacts")
  {
    _value = dartElem->GetElement("max_contacts")->Get<int>();
  }
  else if (_key == "min_step_size")
  {
    _value = dartElem->GetElement("solver")->Get<double>("min_step_size");
  }
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }

  return true;
}

//////////////////////////////////////////////////
bool DARTPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  /// \TODO fill this out, see issue #1115
  try
  {
    if (_key == "solver_type")
    {
      this->SetSolverType(boost::any_cast<std::string>(_value));
    }
    else if (_key == "max_contacts")
    {
      int value = boost::any_cast<int>(_value);
      gzerr << "Setting [" << _key << "] in DART to [" << value
            << "] not yet supported.\n";
    }
    else if (_key == "min_step_size")
    {
      double value = boost::any_cast<double>(_value);
      gzerr << "Setting [" << _key << "] in DART to [" << value
            << "] not yet supported.\n";
    }
    else if (_key == "auto_reset_forces")
    {
      this->dataPtr->resetAllForcesAfterSimulationStep =
          boost::any_cast<bool>(_value);
    }
    else
    {
      // Note: This is nested in the else statement intentionally so that the
      // base PhysicsEngine class will also be updated about the change to the
      // max_step_size parameter.
      if (_key == "max_step_size")
      {
        this->dataPtr->dtWorld->setTimeStep(boost::any_cast<double>(_value));
      }

      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "DARTPhysics::SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
dart::simulation::WorldPtr DARTPhysics::DARTWorld() const
{
  return this->dataPtr->dtWorld;
}

//////////////////////////////////////////////////
void DARTPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::DART);
    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
      msgs::Convert(this->world->MagneticField()));
    physicsMsg.set_enable_physics(this->world->PhysicsEnabled());
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
void DARTPhysics::OnPhysicsMsg(ConstPhysicsPtr& _msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_solver_type())
    this->SetSolverType(_msg->solver_type());

  if (_msg->has_enable_physics())
    this->world->SetPhysicsEnabled(_msg->enable_physics());

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

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
DARTLinkPtr DARTPhysics::FindDARTLink(
    const dart::dynamics::BodyNode *_dtBodyNode)
{
  return StaticFindDARTLink(this, _dtBodyNode);
}

