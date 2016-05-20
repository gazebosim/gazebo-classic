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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/KeyFrame.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/EntityPrivate.hh"
#include "gazebo/physics/Entity.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Entity::Entity(BasePtr _parent)
: Base(*new EntityPrivate, _parent),
  entityDPtr(static_cast<EntityPrivate*>(this->baseDPtr))
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
Entity::Entity(EntityPrivate &_dataPtr, BasePtr _parent)
: Base(_dataPtr, _parent),
  entityDPtr(static_cast<EntityPrivate*>(this->baseDPtr))
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
void Entity::ConstructionHelper()
{
  this->entityDPtr->isStatic = false;
  this->entityDPtr->isCanonicalLink = false;
  this->entityDPtr->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->entityDPtr->visualMsg = new msgs::Visual;
  this->entityDPtr->visualMsg->set_id(this->entityDPtr->id);

  if (this->entityDPtr->world)
    this->entityDPtr->visualMsg->set_parent_name(
        this->entityDPtr->world->Name());
  else
  {
    gzerr << "No world set when constructing an Entity.\n";
    this->entityDPtr->visualMsg->set_parent_name("no_world_name");
  }

  if (this->entityDPtr->parent && this->entityDPtr->parent->HasType(ENTITY))
  {
    this->entityDPtr->parentEntity =
      std::dynamic_pointer_cast<Entity>(this->entityDPtr->parent);
    this->entityDPtr->visualMsg->set_parent_name(
        this->entityDPtr->parentEntity->ScopedName());
    this->SetStatic(this->entityDPtr->parentEntity->IsStatic());
  }

  this->entityDPtr->setWorldPoseFunc = std::bind(
      &Entity::SetWorldPoseDefault, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3);

  this->entityDPtr->scale = ignition::math::Vector3d::One;
}

//////////////////////////////////////////////////
Entity::~Entity()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Entity::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);
  this->entityDPtr->node->Init(this->World()->Name());

  this->entityDPtr->poseSub = this->entityDPtr->node->Subscribe("~/pose/modify",
      &Entity::OnPoseMsg, this);
  this->entityDPtr->visPub =
    this->entityDPtr->node->Advertise<msgs::Visual>("~/visual", 200);
  this->entityDPtr->requestPub =
    this->entityDPtr->node->Advertise<msgs::Request>("~/request");

  this->entityDPtr->visualMsg->set_name(this->ScopedName());

  {
    if (this->entityDPtr->parent && this->entityDPtr->parentEntity)
    {
      this->entityDPtr->worldPose =
        this->entityDPtr->sdf->Get<ignition::math::Pose3d>("pose") +
        this->entityDPtr->parentEntity->WorldPose();
    }
    else
    {
      this->entityDPtr->worldPose =
        this->entityDPtr->sdf->Get<ignition::math::Pose3d>("pose");
    }

    this->entityDPtr->initialRelativePose =
      this->entityDPtr->sdf->Get<ignition::math::Pose3d>("pose");
  }

  if (this->entityDPtr->parent)
  {
    this->entityDPtr->visualMsg->set_parent_name(
        this->entityDPtr->parent->ScopedName());
    this->entityDPtr->visualMsg->set_parent_id(
        this->entityDPtr->parent->Id());
  }
  else
  {
    this->entityDPtr->visualMsg->set_parent_name(
        this->entityDPtr->world->Name());
    this->entityDPtr->visualMsg->set_parent_id(0);
  }
  msgs::Set(this->entityDPtr->visualMsg->mutable_pose(), this->RelativePose());

  if (this->HasType(Base::MODEL))
    this->entityDPtr->visualMsg->set_type(msgs::Visual::MODEL);
  if (this->HasType(Base::LINK))
    this->entityDPtr->visualMsg->set_type(msgs::Visual::LINK);
  if (this->HasType(Base::COLLISION))
    this->entityDPtr->visualMsg->set_type(msgs::Visual::COLLISION);

  this->entityDPtr->visPub->Publish(*this->entityDPtr->visualMsg);

  if (this->HasType(Base::MODEL))
  {
    this->entityDPtr->setWorldPoseFunc = std::bind(&Entity::SetWorldPoseModel,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3);
  }
  else if (this->IsCanonicalLink())
  {
    this->entityDPtr->setWorldPoseFunc = std::bind(
        &Entity::SetWorldPoseCanonicalLink, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  }
  else
  {
    this->entityDPtr->setWorldPoseFunc = std::bind(&Entity::SetWorldPoseDefault,
        this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3);
  }
}

//////////////////////////////////////////////////
void Entity::SetName(const std::string &_name)
{
  // TODO: if an entitie's _name is changed, then the old visual is never
  // removed. Should add in functionality to modify/update the visual
  Base::SetName(_name);
}

//////////////////////////////////////////////////
void Entity::SetStatic(const bool &_s)
{
  Base_V::iterator iter;

  this->entityDPtr->isStatic = _s;

  for (iter = this->entityDPtr->children.begin();
       iter != this->entityDPtr->children.end(); ++iter)
  {
    EntityPtr e = std::dynamic_pointer_cast<Entity>(*iter);
    if (e)
      e->SetStatic(_s);
  }
}

//////////////////////////////////////////////////
bool Entity::IsStatic() const
{
  return this->entityDPtr->isStatic;
}

//////////////////////////////////////////////////
void Entity::SetInitialRelativePose(const math::Pose &_p)
{
  this->SetInitialRelativePose(_p.Ign());
}

//////////////////////////////////////////////////
void Entity::SetInitialRelativePose(const ignition::math::Pose3d &_p)
{
  this->entityDPtr->initialRelativePose = _p;
}

//////////////////////////////////////////////////
math::Pose Entity::GetInitialRelativePose() const
{
  return this->InitialRelativePose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Entity::InitialRelativePose() const
{
  return this->entityDPtr->initialRelativePose;
}

//////////////////////////////////////////////////
math::Box Entity::GetBoundingBox() const
{
  return this->BoundingBox();
}

//////////////////////////////////////////////////
ignition::math::Box Entity::BoundingBox() const
{
  return ignition::math::Box(
      ignition::math::Vector3d::Zero, ignition::math::Vector3d::One);
}

//////////////////////////////////////////////////
void Entity::SetCanonicalLink(const bool _value)
{
  this->entityDPtr->isCanonicalLink = _value;
}

//////////////////////////////////////////////////
void Entity::SetAnimation(common::PoseAnimationPtr _anim)
{
  this->entityDPtr->animationStartPose = this->entityDPtr->worldPose;

  this->entityDPtr->prevAnimationTime = this->entityDPtr->world->SimTime();
  this->entityDPtr->animation = _anim;
  this->entityDPtr->onAnimationComplete = NULL;
  this->entityDPtr->animationConnection =
    event::Events::ConnectWorldUpdateBegin(
        std::bind(&Entity::UpdateAnimation, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void Entity::SetAnimation(const common::PoseAnimationPtr &_anim,
                          std::function<void()> _onComplete)
{
  this->entityDPtr->animationStartPose = this->entityDPtr->worldPose;

  this->entityDPtr->prevAnimationTime = this->entityDPtr->world->SimTime();
  this->entityDPtr->animation = _anim;
  this->entityDPtr->onAnimationComplete = _onComplete;
  this->entityDPtr->animationConnection =
    event::Events::ConnectWorldUpdateBegin(
      std::bind(&Entity::UpdateAnimation, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void Entity::StopAnimation()
{
  this->entityDPtr->animation.reset();
  this->entityDPtr->onAnimationComplete = NULL;
  if (this->entityDPtr->animationConnection)
  {
    event::Events::DisconnectWorldUpdateBegin(
        this->entityDPtr->animationConnection);
    this->entityDPtr->animationConnection.reset();
  }
}

//////////////////////////////////////////////////
void Entity::PublishPose()
{
  GZ_ASSERT(this->ParentModel() != NULL,
      "An entity without a parent model should not happen");

  this->entityDPtr->world->PublishModelPose(ModelPtr(this->ParentModel()));
}

//////////////////////////////////////////////////
math::Pose Entity::GetRelativePose() const
{
  return this->RelativePose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Entity::RelativePose() const
{
  // We return the initialRelativePose for COLLISION objects because they
  // cannot move relative to their parent link.
  // \todo Look into storing relative poses for all objects instead of world
  // poses. It may simplify pose updating.
  if (this->IsCanonicalLink() || this->HasType(COLLISION))
  {
    return this->entityDPtr->initialRelativePose;
  }
  else if (this->entityDPtr->parent && this->entityDPtr->parentEntity)
  {
    return this->entityDPtr->worldPose -
      this->entityDPtr->parentEntity->WorldPose();
  }
  else
  {
    return this->entityDPtr->worldPose;
  }
}

//////////////////////////////////////////////////
void Entity::SetRelativePose(const math::Pose &_pose, bool _notify,
        bool _publish)
{
  this->SetRelativePose(_pose.Ign(), _notify, _publish);
}

//////////////////////////////////////////////////
void Entity::SetRelativePose(const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  if (this->entityDPtr->parent && this->entityDPtr->parentEntity)
  {
    this->SetWorldPose(_pose +
        this->entityDPtr->parentEntity->WorldPose(), _notify, _publish);
  }
  else
  {
    this->SetWorldPose(_pose, _notify, _publish);
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldTwist(const math::Vector3 &_linear,
    const math::Vector3 &_angular, bool _updateChildren)
{
  this->SetWorldTwist(_linear.Ign(), _angular.Ign(), _updateChildren);
}

//////////////////////////////////////////////////
void Entity::SetWorldTwist(const ignition::math::Vector3d &_linear,
    const ignition::math::Vector3d &_angular, const bool _updateChildren)
{
  if (this->HasType(LINK) || this->HasType(MODEL))
  {
    if (this->HasType(LINK))
    {
      Link* link = dynamic_cast<Link*>(this);
      link->SetLinearVel(_linear);
      link->SetAngularVel(_angular);
    }
    if (_updateChildren)
    {
      // force an update of all children
      for  (Base_V::iterator iter = this->entityDPtr->children.begin();
            iter != this->entityDPtr->children.end(); ++iter)
      {
        if ((*iter)->HasType(ENTITY))
        {
          EntityPtr entity = std::static_pointer_cast<Entity>(*iter);
          entity->SetWorldTwist(_linear, _angular, _updateChildren);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseModel(const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  ignition::math::Pose3d oldModelWorldPose = this->entityDPtr->worldPose;

  // initialization: (no children?) set own worldPose
  this->entityDPtr->worldPose = _pose;
  this->entityDPtr->worldPose.Correct();

  // (OnPoseChange uses GetWorldPose)
  if (_notify)
    this->UpdatePhysicsPose(false);

  //
  // user deliberate setting: lock and update all children's wp
  //

  // force an update of all children
  // update all children pose, moving them with the model.
  // The outer loop updates all the links.
  for (Base_V::iterator iter = this->entityDPtr->children.begin();
      iter != this->entityDPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(ENTITY))
    {
      EntityPtr entity = std::static_pointer_cast<Entity>(*iter);

      if (entity->HasType(LINK))
      {
        if (entity->IsCanonicalLink())
        {
          entity->entityDPtr->worldPose =
            (entity->entityDPtr->initialRelativePose + _pose);
        }
        else
        {
          entity->entityDPtr->worldPose =
            ((entity->entityDPtr->worldPose - oldModelWorldPose) + _pose);
          if (_publish)
            entity->PublishPose();
        }

        if (_notify)
          entity->UpdatePhysicsPose(false);

        // Tell collisions that their current world pose is dirty (needs
        // updating). We set a dirty flag instead of directly updating the
        // value to improve performance.
        for (unsigned int childIndex = 0;
             childIndex < (*iter)->ChildCount(); ++childIndex)
        {
          if ((*iter)->Child(childIndex)->HasType(COLLISION))
          {
            CollisionPtr entityC =
                std::static_pointer_cast<Collision>((*iter)->Child(childIndex));
            entityC->SetWorldPoseDirty();
          }
        }
      }
      else if (entity->HasType(MODEL))
      {
        // set pose of nested models
        entity->SetWorldPoseModel(
            (entity->entityDPtr->worldPose - oldModelWorldPose) + _pose,
            _notify, _publish);
      }
      else
      {
        gzerr << "SetWorldPoseModel error: unknown type of entity in Model."
          << std::endl;
      }
    }
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseCanonicalLink(
    const ignition::math::Pose3d &_pose, const bool _notify,
    const bool _publish)
{
  this->entityDPtr->worldPose = _pose;
  this->entityDPtr->worldPose.Correct();

  if (_notify)
    this->UpdatePhysicsPose(true);

  if (!this->entityDPtr->parentEntity->HasType(MODEL))
  {
    gzerr << "SetWorldPose for Canonical Body [" << this->Name()
        << "] but parent[" << this->entityDPtr->parentEntity->Name()
        << "] is not a MODEL!" << std::endl;
    return;
  }

  EntityPtr parentEnt = this->entityDPtr->parentEntity;
  ignition::math::Pose3d relativePose = this->entityDPtr->initialRelativePose;
  ignition::math::Pose3d updatePose = _pose;

  // recursively update parent model pose based on new canonical link pose
  while (parentEnt && parentEnt->HasType(MODEL))
  {
    // setting parent Model world pose from canonical link world pose
    // where _pose is the canonical link's world pose
    parentEnt->entityDPtr->worldPose = -relativePose + updatePose;

    parentEnt->entityDPtr->worldPose.Correct();

    if (_notify)
      parentEnt->UpdatePhysicsPose(false);

    if (_publish)
      this->entityDPtr->parentEntity->PublishPose();

    updatePose = parentEnt->entityDPtr->worldPose;
    relativePose = parentEnt->InitialRelativePose();

    parentEnt = std::dynamic_pointer_cast<Entity>(parentEnt->Parent());
  }

  // Tell collisions that their current world pose is dirty (needs
  // updating). We set a dirty flag instead of directly updating the
  // value to improve performance.
  for (Base_V::iterator iterC = this->entityDPtr->children.begin();
      iterC != this->entityDPtr->children.end(); ++iterC)
  {
    if ((*iterC)->HasType(COLLISION))
    {
      CollisionPtr entityC = std::static_pointer_cast<Collision>(*iterC);
      entityC->SetWorldPoseDirty();
    }
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseDefault(
    const ignition::math::Pose3d &_pose, const bool _notify,
    const bool /*_publish*/)
{
  this->entityDPtr->worldPose = _pose;
  this->entityDPtr->worldPose.Correct();

  if (_notify)
    this->UpdatePhysicsPose(true);
}

//////////////////////////////////////////////////
void Entity::SetWorldPose(const math::Pose &_pose, bool _notify, bool _publish)
{
  this->SetWorldPose(_pose.Ign(), _notify, _publish);
}

//////////////////////////////////////////////////
//   The entity stores an initialRelativePose and dynamic worldPose
//   When calling SetWorldPose (SWP) or SetRelativePose on an entity
//   that is a Model (M), Canonical Body (CB) or Body (B), different
//   considerations need to be taken.
// Below is a table that summarizes the current code.
//  +----------------------------------------------------------+
//  |     |     M          |  CB             |  B              |
//  |----------------------------------------------------------|
//  |SWP  | Lock           | Lock            | Set BWP         |
//  |     | Update MWP     | Set CBWP        |                 |
//  |     | SWP Children   | SWP M = CB-CBRP |                 |
//  |----------------------------------------------------------|
//  |SRP  | WP = RP + PP   | WP = RP + PP    | WP = RP + PP    |
//  |----------------------------------------------------------|
//  |GWP  | return WP      | return WP       | return WP       |
//  |----------------------------------------------------------|
//  |GRP  | RP = WP - RP   | return CBRP     | RP = WP - RP    |
//  +----------------------------------------------------------+
//  Legends
//    M    - Model
//    CB   - Canonical Body
//    B    - Non-Canonical Body
//    *WP  - *WorldPose
//    *RP  - *RelativePose (relative to parent)
//    SWP  - SetWorldPose
//    GWP  - GetWorldPose
//    MWP  - Model World Pose
//    CBRP - Canonical Body Relative (to Model) Pose
//
void Entity::SetWorldPose(const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  {
    std::lock_guard<std::mutex> (this->World()->WorldPoseMutex());
    // (*this.*setWorldPoseFunc)(_pose, _notify, _publish);
    this->entityDPtr->setWorldPoseFunc(_pose, _notify, _publish);
  }
  if (_publish)
    this->PublishPose();
}

//////////////////////////////////////////////////
void Entity::UpdatePhysicsPose(const bool _updateChildren)
{
  this->OnPoseChange();

  /// if children update is requested
  if (_updateChildren)
  {
    for (Base_V::iterator iter = this->entityDPtr->children.begin();
         iter != this->entityDPtr->children.end(); ++iter)
    {
      if ((*iter)->HasType(LINK))
      {
        // call child Link::OnPoseChange()
        std::static_pointer_cast<Link>(*iter)->OnPoseChange();
      }
    }
  }

  /// Static Collision objects have no corresponding ODE body
  /// So if one calls MyStaticModel.SetWorldPose(p),
  /// we should force children update
  if (this->IsStatic())
  {
    for (Base_V::iterator iter = this->entityDPtr->children.begin();
         iter != this->entityDPtr->children.end(); ++iter)
    {
      CollisionPtr coll = std::static_pointer_cast<Collision>(*iter);
      if (coll && (*iter)->HasType(COLLISION))
      {
        // update collision pose
        //   to model's world pose + it's intial relative pose
        coll->entityDPtr->worldPose.Pos() = this->entityDPtr->worldPose.Pos() +
          this->entityDPtr->worldPose.Rot().RotateVector(
              coll->entityDPtr->initialRelativePose.Pos());
        coll->entityDPtr->worldPose.Rot() = this->entityDPtr->worldPose.Rot() *
          coll->entityDPtr->initialRelativePose.Rot();
        coll->OnPoseChange();
      }
      else
      {
        // not static or not a Collision type, do nothing
      }
    }
  }
}

//////////////////////////////////////////////////
ModelPtr Entity::GetParentModel()
{
  return ModelPtr(this->ParentModel());
}

//////////////////////////////////////////////////
Model *Entity::ParentModel()
{
  if (this->HasType(MODEL))
    return dynamic_cast<Model*>(this);

  BasePtr p = this->entityDPtr->parent;
  GZ_ASSERT(p, "Parent of an entity is NULL");

  while (p->Parent() && p->Parent()->HasType(MODEL))
    p = p->Parent();

  return dynamic_cast<Model*>(p.get());
}

//////////////////////////////////////////////////
CollisionPtr Entity::GetChildCollision(const std::string &_name)
{
  return this->ChildCollision(_name);
}

//////////////////////////////////////////////////
CollisionPtr Entity::ChildCollision(const std::string &_name) const
{
  Base *base = this->BaseByName(_name);
  if (base)
    return CollisionPtr(static_cast<Collision*>(base));

  return CollisionPtr();
}

//////////////////////////////////////////////////
LinkPtr Entity::GetChildLink(const std::string &_name)
{
  return this->ChildLink(_name);
}

//////////////////////////////////////////////////
LinkPtr Entity::ChildLink(const std::string &_name) const
{
  Base *base = this->BaseByName(_name);
  if (base)
    return LinkPtr(static_cast<Link*>(base));

  return LinkPtr();
}

//////////////////////////////////////////////////
void Entity::OnPoseMsg(ConstPosePtr &_msg)
{
  if (_msg->name() == this->ScopedName())
  {
    ignition::math::Pose3d p = msgs::ConvertIgn(*_msg);
    this->SetWorldPose(p);
  }
}

//////////////////////////////////////////////////
void Entity::Fini()
{
  // TODO: put this back in
  // this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  if (this->entityDPtr->requestPub)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete", this->GetScopedName());
    this->entityDPtr->requestPub->Publish(*msg, true);
    delete msg;
  }

  // Clean transport
  {
    this->entityDPtr->posePub.reset();
    this->entityDPtr->requestPub.reset();
    this->entityDPtr->visPub.reset();

    this->entityDPtr->poseSub.reset();

    if (this->entityDPtr->node)
      this->entityDPtr->node->Fini();
    this->entityDPtr->node.reset();
  }

  this->entityDPtr->animationConnection.reset();
  this->entityDPtr->connections.clear();

  if (this->entityDPtr->visualMsg)
    delete this->entityDPtr->visualMsg;
  this->entityDPtr->visualMsg = NULL;

  this->parentEntity.reset();

  Base::Fini();
}

//////////////////////////////////////////////////
void Entity::Reset()
{
  this->SetRelativePose(this->entityDPtr->initialRelativePose);
}

//////////////////////////////////////////////////
void Entity::UpdateParameters(sdf::ElementPtr _sdf)
{
  Base::UpdateParameters(_sdf);

  ignition::math::Pose3d parentPose;
  if (this->entityDPtr->parent && this->entityDPtr->parentEntity)
    parentPose = this->entityDPtr->parentEntity->entityDPtr->worldPose;

  ignition::math::Pose3d newPose = _sdf->Get<ignition::math::Pose3d>("pose");
  if (newPose != this->RelativePose())
  {
    this->SetRelativePose(newPose);
  }
}

//////////////////////////////////////////////////
void Entity::UpdateAnimation(const common::UpdateInfo &_info)
{
  common::PoseKeyFrame kf(0);

  this->entityDPtr->animation->AddTime(
      (_info.simTime - this->entityDPtr->prevAnimationTime).Double());
  this->entityDPtr->animation->GetInterpolatedKeyFrame(kf);

  ignition::math::Pose3d offset;
  offset.Pos() = kf.Translation();
  offset.Rot() = kf.Rotation();

  this->SetWorldPose(offset);
  this->entityDPtr->prevAnimationTime = _info.simTime;

  if (this->entityDPtr->animation->GetLength() <=
      this->entityDPtr->animation->GetTime())
  {
    event::Events::DisconnectWorldUpdateBegin(
        this->entityDPtr->animationConnection);
    this->entityDPtr->animationConnection.reset();
    if (this->entityDPtr->onAnimationComplete)
    {
      this->entityDPtr->onAnimationComplete();
    }
  }
}

//////////////////////////////////////////////////
math::Pose Entity::GetDirtyPose() const
{
  return this->DirtyPose();
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Entity::DirtyPose() const
{
  return this->entityDPtr->dirtyPose;
}

//////////////////////////////////////////////////
math::Box Entity::GetCollisionBoundingBox() const
{
  return this->CollisionBoundingBox();
}

//////////////////////////////////////////////////
ignition::math::Box Entity::CollisionBoundingBox() const
{
  BasePtr base = std::const_pointer_cast<Base>(shared_from_this());
  return this->CollisionBoundingBoxHelper(base);
}

//////////////////////////////////////////////////
ignition::math::Box Entity::CollisionBoundingBoxHelper(BasePtr _base) const
{
  if (_base->HasType(COLLISION))
    return std::dynamic_pointer_cast<Collision>(_base)->BoundingBox();

  ignition::math::Box box;

  for (unsigned int i = 0; i < _base->ChildCount(); i++)
  {
    box += this->CollisionBoundingBoxHelper(_base->Child(i));
  }

  return box;
}

//////////////////////////////////////////////////
void Entity::PlaceOnEntity(const std::string &_entityName)
{
  EntityPtr onEntity = this->World()->EntityByName(_entityName);
  ignition::math::Box box = this->CollisionBoundingBox();
  ignition::math::Box onBox = onEntity->CollisionBoundingBox();

  ignition::math::Pose3d p = onEntity->WorldPose();
  p.Pos().Z() = onBox.Max().Z() + box.ZLength()*0.5;
  this->SetWorldPose(p);
}

//////////////////////////////////////////////////
void Entity::GetNearestEntityBelow(double &_distBelow,
                                   std::string &_entityName)
{
  return this->NearestEntityBelow(_distBelow, _entityName);
}

//////////////////////////////////////////////////
void Entity::NearestEntityBelow(double &_distBelow,
                                   std::string &_entityName) const
{
  this->World()->Physics()->InitForThread();
  RayShapePtr rayShape = std::dynamic_pointer_cast<RayShape>(
    this->World()->Physics()->CreateShape("ray", CollisionPtr()));

  ignition::math::Box box = this->CollisionBoundingBox();
  ignition::math::Vector3d start = this->WorldPose().Pos();
  ignition::math::Vector3d end = start;
  start.Z() = box.Min().Z() - 0.00001;
  end.Z() -= 1000;
  rayShape->SetPoints(start, end);
  rayShape->Intersection(_distBelow, _entityName);
  _distBelow += 0.00001;
}

//////////////////////////////////////////////////
void Entity::PlaceOnNearestEntityBelow()
{
  double dist;
  std::string entityName;
  this->NearestEntityBelow(dist, entityName);
  if (dist > 0.0)
  {
    ignition::math::Pose3d p = this->WorldPose();
    p.Pos().Z() -= dist;
    this->SetWorldPose(p);
  }
}

//////////////////////////////////////////////////
math::Pose Entity::GetWorldPose() const
{
  return this->WorldPose();
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Entity::WorldPose() const
{
  return this->entityDPtr->worldPose;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetRelativeLinearVel() const
{
  return this->RelativeLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeLinearVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetWorldLinearVel() const
{
  return this->WorldLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldLinearVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetRelativeAngularVel() const
{
  return this->RelativeAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeAngularVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetWorldAngularVel() const
{
  return this->WorldAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldAngularVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetRelativeLinearAccel() const
{
  return this->RelativeLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeLinearAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetWorldLinearAccel() const
{
  return this->WorldLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldLinearAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetRelativeAngularAccel() const
{
  return this->RelativeAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeAngularAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Entity::GetWorldAngularAccel() const
{
  return this->WorldAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldAngularAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
bool Entity::IsCanonicalLink() const
{
  return this->entityDPtr->isCanonicalLink;
}

