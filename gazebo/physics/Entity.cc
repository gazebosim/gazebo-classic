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
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>

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
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Entity.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Entity::Entity(BasePtr _parent)
  : Base(_parent)
{
  this->isStatic = false;
  this->isCanonicalLink = false;
  this->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->visualMsg = new msgs::Visual;
  this->visualMsg->set_id(this->id);

  if (this->world)
    this->visualMsg->set_parent_name(this->world->Name());
  else
  {
    gzerr << "No world set when constructing an Entity.\n";
    this->visualMsg->set_parent_name("no_world_name");
  }

  if (this->parent && this->parent->HasType(ENTITY))
  {
    this->parentEntity = boost::dynamic_pointer_cast<Entity>(this->parent);
    this->visualMsg->set_parent_name(this->parentEntity->GetScopedName());
    this->SetStatic(this->parentEntity->IsStatic());
  }

  this->setWorldPoseFunc = &Entity::SetWorldPoseDefault;

  this->scale = ignition::math::Vector3d::One;
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
  this->node->Init(this->GetWorld()->Name());

  this->poseSub = this->node->Subscribe("~/pose/modify",
      &Entity::OnPoseMsg, this);
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 200);
  this->requestPub = this->node->Advertise<msgs::Request>("~/request");

  this->visualMsg->set_name(this->GetScopedName());

  {
    if (this->parent && this->parentEntity)
    {
      this->worldPose = this->sdf->Get<ignition::math::Pose3d>("pose") +
                        this->parentEntity->worldPose;
    }
    else
    {
      this->worldPose = this->sdf->Get<ignition::math::Pose3d>("pose");
    }

    this->initialRelativePose = this->sdf->Get<ignition::math::Pose3d>("pose");
  }

  if (this->parent)
  {
    this->visualMsg->set_parent_name(this->parent->GetScopedName());
    this->visualMsg->set_parent_id(this->parent->GetId());
  }
  else
  {
    this->visualMsg->set_parent_name(this->world->Name());
    this->visualMsg->set_parent_id(0);
  }
  msgs::Set(this->visualMsg->mutable_pose(), this->RelativePose());

  if (this->HasType(Base::MODEL))
    this->visualMsg->set_type(msgs::Visual::MODEL);
  if (this->HasType(Base::LINK))
    this->visualMsg->set_type(msgs::Visual::LINK);
  if (this->HasType(Base::COLLISION))
    this->visualMsg->set_type(msgs::Visual::COLLISION);

  this->visPub->Publish(*this->visualMsg);

  if (this->HasType(Base::MODEL))
    this->setWorldPoseFunc = &Entity::SetWorldPoseModel;
  else if (this->IsCanonicalLink())
    this->setWorldPoseFunc = &Entity::SetWorldPoseCanonicalLink;
  else
    this->setWorldPoseFunc = &Entity::SetWorldPoseDefault;
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

  this->isStatic = _s;

  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    EntityPtr e = boost::dynamic_pointer_cast<Entity>(*iter);
    if (e)
      e->SetStatic(_s);
  }
}

//////////////////////////////////////////////////
bool Entity::IsStatic() const
{
  return this->isStatic;
}

//////////////////////////////////////////////////
void Entity::SetInitialRelativePose(const ignition::math::Pose3d &_p)
{
  this->initialRelativePose = _p;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Entity::InitialRelativePose() const
{
  return this->initialRelativePose;
}


//////////////////////////////////////////////////
ignition::math::Box Entity::BoundingBox() const
{
  return ignition::math::Box(
      ignition::math::Vector3d::Zero, ignition::math::Vector3d::One);
}

//////////////////////////////////////////////////
void Entity::SetCanonicalLink(bool _value)
{
  this->isCanonicalLink = _value;
}

//////////////////////////////////////////////////
void Entity::SetAnimation(common::PoseAnimationPtr _anim)
{
  this->animationStartPose = this->worldPose;

  this->prevAnimationTime = this->world->SimTime();
  this->animation = _anim;
  this->onAnimationComplete.clear();
  this->animationConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Entity::UpdateAnimation, this, _1));
}

//////////////////////////////////////////////////
void Entity::SetAnimation(const common::PoseAnimationPtr &_anim,
                          boost::function<void()> _onComplete)
{
  this->animationStartPose = this->worldPose;

  this->prevAnimationTime = this->world->SimTime();
  this->animation = _anim;
  this->onAnimationComplete = _onComplete;
  this->animationConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Entity::UpdateAnimation, this, _1));
}

//////////////////////////////////////////////////
void Entity::StopAnimation()
{
  this->animation.reset();
  this->onAnimationComplete.clear();
  this->animationConnection.reset();
}

//////////////////////////////////////////////////
void Entity::PublishPose()
{
  GZ_ASSERT(this->GetParentModel() != NULL,
      "An entity without a parent model should not happen");

  this->world->PublishModelPose(this->GetParentModel());
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
    return this->initialRelativePose;
  }
  else if (this->parent && this->parentEntity)
  {
    return this->WorldPose() - this->parentEntity->WorldPose();
  }
  else
  {
    return this->WorldPose();
  }
}

//////////////////////////////////////////////////
void Entity::SetRelativePose(const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  if (this->parent && this->parentEntity)
    this->SetWorldPose(_pose + this->parentEntity->WorldPose(), _notify,
                              _publish);
  else
    this->SetWorldPose(_pose, _notify, _publish);
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
      for  (Base_V::iterator iter = this->children.begin();
            iter != this->children.end(); ++iter)
      {
        if ((*iter)->HasType(ENTITY))
        {
          EntityPtr entity = boost::static_pointer_cast<Entity>(*iter);
          entity->SetWorldTwist(_linear, _angular, _updateChildren);
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseModel(
    const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  ignition::math::Pose3d oldModelWorldPose = this->worldPose;

  // initialization: (no children?) set own worldPose
  this->worldPose = _pose;
  this->worldPose.Correct();

  // (OnPoseChange uses GetWorldPose)
  if (_notify)
    this->UpdatePhysicsPose(false);

  //
  // user deliberate setting: lock and update all children's wp
  //

  // force an update of all children
  // update all children pose, moving them with the model.
  // The outer loop updates all the links.
  for (Base_V::iterator iter = this->children.begin();
      iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(ENTITY))
    {
      EntityPtr entity = boost::static_pointer_cast<Entity>(*iter);

      if (entity->HasType(LINK))
      {
        if (entity->IsCanonicalLink())
          entity->worldPose = (entity->initialRelativePose + _pose);
        else
        {
          entity->worldPose = ((entity->worldPose - oldModelWorldPose) + _pose);

          // Publish only for non-canonical links,
          // since local pose of the canonical link does not change,
          // so there is no need to keep telling everyone about it.
          // The canonical link will automatically move as the model moves
          if (_publish)
            entity->PublishPose();
        }

        if (_notify)
          entity->UpdatePhysicsPose(false);

        // Tell collisions that their current world pose is dirty (needs
        // updating). We set a dirty flag instead of directly updating the
        // value to improve performance.
        for (Base_V::iterator iterC = (*iter)->children.begin();
             iterC != (*iter)->children.end(); ++iterC)
        {
          if ((*iterC)->HasType(COLLISION))
          {
            CollisionPtr entityC =
                boost::static_pointer_cast<Collision>(*iterC);
            entityC->SetWorldPoseDirty();
          }
          else if ((*iterC)->HasType(LIGHT))
          {
            LightPtr entityC =
                boost::static_pointer_cast<Light>(*iterC);
            entityC->SetWorldPoseDirty();
          }
        }
      }
      else if (entity->HasType(MODEL))
      {
        // set pose of nested models
        entity->SetWorldPoseModel(
            (entity->worldPose - oldModelWorldPose) + _pose, _notify, _publish);
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
    const ignition::math::Pose3d &_pose,
    const bool _notify, const bool _publish)
{
  this->SetWorldPoseDefault(_pose, _notify, _publish);

  if (!this->parentEntity->HasType(MODEL))
  {
    gzerr << "SetWorldPose for Canonical Body [" << this->GetName()
        << "] but parent[" << this->parentEntity->GetName()
        << "] is not a MODEL!" << std::endl;
    return;
  }

  EntityPtr parentEnt = this->parentEntity;
  ignition::math::Pose3d relativePose = this->initialRelativePose;
  ignition::math::Pose3d updatePose = _pose;

  // recursively update parent model pose based on new canonical link pose
  while (parentEnt && parentEnt->HasType(MODEL))
  {
    // setting parent Model world pose from canonical link world pose
    // where _pose is the canonical link's world pose
    parentEnt->worldPose = ignition::math::Pose3d(-relativePose) + updatePose;

    parentEnt->worldPose.Correct();

    if (_notify)
      parentEnt->UpdatePhysicsPose(false);

    if (_publish)
      this->parentEntity->PublishPose();

    updatePose = parentEnt->worldPose;
    relativePose = parentEnt->InitialRelativePose();

    parentEnt = boost::dynamic_pointer_cast<Entity>(parentEnt->GetParent());
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseDefault(const ignition::math::Pose3d &_pose,
    const bool _notify, const bool /*_publish*/)
{
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify)
    this->UpdatePhysicsPose(true);

  if (this->HasType(LINK))
  {
    // Tell collisions that their current world pose is dirty (needs
    // updating). We set a dirty flag instead of directly updating the
    // value to improve performance.
    for (auto &childPtr : this->children)
    {
      if (childPtr->HasType(COLLISION))
      {
        CollisionPtr entityC = boost::static_pointer_cast<Collision>(childPtr);
        entityC->SetWorldPoseDirty();
      }
      else if (childPtr->HasType(LIGHT))
      {
        LightPtr entityC = boost::static_pointer_cast<Light>(childPtr);
        entityC->SetWorldPoseDirty();
      }
    }
  }
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
    std::lock_guard<std::mutex> lock(this->GetWorld()->WorldPoseMutex());
    (*this.*setWorldPoseFunc)(_pose, _notify, _publish);
  }
  if (_publish)
    this->PublishPose();
}

//////////////////////////////////////////////////
void Entity::UpdatePhysicsPose(bool _updateChildren)
{
  this->OnPoseChange();

  /// if children update is requested
  if (_updateChildren)
  {
    for (Base_V::iterator iter = this->children.begin();
         iter != this->children.end(); ++iter)
    {
      if ((*iter)->HasType(LINK))
      {
        // call child Link::OnPoseChange()
        boost::static_pointer_cast<Link>(*iter)->OnPoseChange();
      }
    }
  }

  /// Static Collision objects have no corresponding ODE body
  /// So if one calls MyStaticModel.SetWorldPose(p),
  /// we should force children update
  if (this->IsStatic())
  {
    for (Base_V::iterator iter = this->children.begin();
         iter != this->children.end(); ++iter)
    {
      CollisionPtr coll = boost::static_pointer_cast<Collision>(*iter);
      if (coll && (*iter)->HasType(COLLISION))
      {
        // update collision pose
        //   to model's world pose + it's intial relative pose
        coll->worldPose.Pos() = this->worldPose.Pos() +
          this->worldPose.Rot().RotateVector(coll->initialRelativePose.Pos());
        coll->worldPose.Rot() = this->worldPose.Rot() *
          coll->initialRelativePose.Rot();
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
  BasePtr p;
  if (this->HasType(MODEL))
    return boost::dynamic_pointer_cast<Model>(shared_from_this());

  p = this->parent;
  GZ_ASSERT(p, "Parent of an entity is NULL");

  while (p->GetParent() && p->GetParent()->HasType(MODEL))
    p = p->GetParent();

  return boost::dynamic_pointer_cast<Model>(p);
}

//////////////////////////////////////////////////
CollisionPtr Entity::GetChildCollision(const std::string &_name)
{
  BasePtr base = this->GetByName(_name);
  if (base)
    return boost::dynamic_pointer_cast<Collision>(base);

  return CollisionPtr();
}

//////////////////////////////////////////////////
LinkPtr Entity::GetChildLink(const std::string &_name)
{
  BasePtr base = this->GetByName(_name);
  if (base)
    return boost::dynamic_pointer_cast<Link>(base);

  return LinkPtr();
}

//////////////////////////////////////////////////
void Entity::OnPoseMsg(ConstPosePtr &_msg)
{
  if (_msg->name() == this->GetScopedName())
  {
    ignition::math::Pose3d p = msgs::ConvertIgn(*_msg);
    this->SetWorldPose(p);
  }
}

//////////////////////////////////////////////////
void Entity::Fini()
{
  // TODO: put this back in
  // this->GetWorld()-Physics()->RemoveEntity(this);

  if (this->requestPub)
  {
    auto msg = msgs::CreateRequest("entity_delete", this->GetScopedName());
    this->requestPub->Publish(*msg, true);
    delete msg;
  }

  // Clean transport
  {
    this->posePub.reset();
    this->requestPub.reset();
    this->visPub.reset();

    this->poseSub.reset();

    if (this->node)
      this->node->Fini();
    this->node.reset();
  }

  this->animationConnection.reset();
  this->connections.clear();

  if (this->visualMsg)
    delete this->visualMsg;
  this->visualMsg = NULL;

  this->parentEntity.reset();

  Base::Fini();
}

//////////////////////////////////////////////////
void Entity::Reset()
{
  this->SetRelativePose(this->initialRelativePose);
}

//////////////////////////////////////////////////
void Entity::UpdateParameters(sdf::ElementPtr _sdf)
{
  Base::UpdateParameters(_sdf);

  ignition::math::Pose3d parentPose;
  if (this->parent && this->parentEntity)
    parentPose = this->parentEntity->worldPose;

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

  this->animation->AddTime((_info.simTime - this->prevAnimationTime).Double());
  this->animation->GetInterpolatedKeyFrame(kf);

  ignition::math::Pose3d offset;
  offset.Pos() = kf.Translation();
  offset.Rot() = kf.Rotation();

  this->SetWorldPose(offset);
  this->prevAnimationTime = _info.simTime;

  if (this->animation->GetLength() <= this->animation->GetTime())
  {
    this->animationConnection.reset();
    if (this->onAnimationComplete)
    {
      this->onAnimationComplete();
    }
  }
}

//////////////////////////////////////////////////
const ignition::math::Pose3d &Entity::DirtyPose() const
{
  return this->dirtyPose;
}

//////////////////////////////////////////////////
ignition::math::Box Entity::CollisionBoundingBox() const
{
  BasePtr base = boost::const_pointer_cast<Base>(shared_from_this());
  return this->CollisionBoundingBoxHelper(base);
}

//////////////////////////////////////////////////
ignition::math::Box Entity::CollisionBoundingBoxHelper(BasePtr _base) const
{
  if (_base->HasType(COLLISION))
    return boost::dynamic_pointer_cast<Collision>(_base)->BoundingBox();

  ignition::math::Box box;

  for (unsigned int i = 0; i < _base->GetChildCount(); i++)
  {
    box += this->CollisionBoundingBoxHelper(_base->GetChild(i));
  }

  return box;
}

//////////////////////////////////////////////////
void Entity::PlaceOnEntity(const std::string &_entityName)
{
  EntityPtr onEntity = this->GetWorld()->EntityByName(_entityName);
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
  this->GetWorld()->Physics()->InitForThread();
  RayShapePtr rayShape = boost::dynamic_pointer_cast<RayShape>(
    this->GetWorld()->Physics()->CreateShape("ray", CollisionPtr()));

  ignition::math::Box box = this->CollisionBoundingBox();
  ignition::math::Vector3d start = this->WorldPose().Pos();
  ignition::math::Vector3d end = start;
  start.Z() = box.Min().Z() - 0.00001;
  end.Z() -= 1000;
  rayShape->SetPoints(start, end);
  rayShape->GetIntersection(_distBelow, _entityName);
  _distBelow += 0.00001;
}

//////////////////////////////////////////////////
void Entity::PlaceOnNearestEntityBelow()
{
  double dist;
  std::string entityName;
  this->GetNearestEntityBelow(dist, entityName);
  if (dist > 0.0)
  {
    ignition::math::Pose3d p = this->WorldPose();
    p.Pos().Z() -= dist;
    this->SetWorldPose(p);
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeLinearVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldLinearVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeAngularVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldAngularVel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeLinearAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldLinearAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::RelativeAngularAccel() const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Entity::WorldAngularAccel() const
{
  return ignition::math::Vector3d::Zero;
}
