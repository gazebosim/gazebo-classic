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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

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
  this->isCanonicalLink = false;
  this->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->visualMsg = new msgs::Visual;
  this->visualMsg->set_id(this->id);

  if (this->world)
    this->visualMsg->set_parent_name(this->world->GetName());
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

  this->scale = math::Vector3::One;
}

//////////////////////////////////////////////////
Entity::~Entity()
{
  // TODO: put this back in
  // this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  delete this->visualMsg;
  this->visualMsg = NULL;

  this->visPub.reset();
  this->requestPub.reset();
  this->poseSub.reset();
  this->node.reset();
}

//////////////////////////////////////////////////
void Entity::Load(sdf::ElementPtr _sdf)
{
  Base::Load(_sdf);
  this->node->Init(this->GetWorld()->GetName());

  this->poseSub = this->node->Subscribe("~/pose/modify",
      &Entity::OnPoseMsg, this);
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 200);
  this->requestPub = this->node->Advertise<msgs::Request>("~/request");

  this->visualMsg->set_name(this->GetScopedName());

  {
    if (this->parent && this->parentEntity)
      this->worldPose = this->sdf->Get<math::Pose>("pose") +
                        this->parentEntity->worldPose;
    else
      this->worldPose = this->sdf->Get<math::Pose>("pose");

    this->initialRelativePose = this->sdf->Get<math::Pose>("pose");
  }

  if (this->parent)
  {
    this->visualMsg->set_parent_name(this->parent->GetScopedName());
    this->visualMsg->set_parent_id(this->parent->GetId());
  }
  else
  {
    this->visualMsg->set_parent_name(this->world->GetName());
    this->visualMsg->set_parent_id(0);
  }
  msgs::Set(this->visualMsg->mutable_pose(), this->GetRelativePose().Ign());

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
void Entity::SetInitialRelativePose(const math::Pose &_p)
{
  this->initialRelativePose = _p;
}

//////////////////////////////////////////////////
math::Pose Entity::GetInitialRelativePose() const
{
  return this->initialRelativePose;
}

//////////////////////////////////////////////////
math::Box Entity::GetBoundingBox() const
{
  return math::Box(math::Vector3(0, 0, 0), math::Vector3(1, 1, 1));
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

  this->prevAnimationTime = this->world->GetSimTime();
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

  this->prevAnimationTime = this->world->GetSimTime();
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
  if (this->animationConnection)
  {
    event::Events::DisconnectWorldUpdateBegin(this->animationConnection);
    this->animationConnection.reset();
  }
}

//////////////////////////////////////////////////
void Entity::PublishPose()
{
  GZ_ASSERT(this->GetParentModel() != NULL,
      "An entity without a parent model should not happen");

  this->world->PublishModelPose(this->GetParentModel());
}

//////////////////////////////////////////////////
math::Pose Entity::GetRelativePose() const
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
    return this->worldPose - this->parentEntity->GetWorldPose();
  }
  else
  {
    return this->worldPose;
  }
}

//////////////////////////////////////////////////
void Entity::SetRelativePose(const math::Pose &_pose, bool _notify,
        bool _publish)
{
  if (this->parent && this->parentEntity)
    this->SetWorldPose(_pose + this->parentEntity->GetWorldPose(), _notify,
                              _publish);
  else
    this->SetWorldPose(_pose, _notify, _publish);
}

//////////////////////////////////////////////////
void Entity::SetWorldTwist(const math::Vector3 &_linear,
    const math::Vector3 &_angular, bool _updateChildren)
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
void Entity::SetWorldPoseModel(const math::Pose &_pose, bool _notify,
        bool _publish)
{
  math::Pose oldModelWorldPose = this->worldPose;

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

      if (entity->IsCanonicalLink())
        entity->worldPose = (entity->initialRelativePose + _pose);
      else
      {
        entity->worldPose = ((entity->worldPose - oldModelWorldPose) + _pose);
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
          CollisionPtr entityC = boost::static_pointer_cast<Collision>(*iterC);
          entityC->SetWorldPoseDirty();
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseCanonicalLink(const math::Pose &_pose, bool _notify,
        bool _publish)
{
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify)
    this->UpdatePhysicsPose(true);

  // also update parent model's pose
  if (this->parentEntity->HasType(MODEL))
  {
    // setting parent Model world pose from canonical link world pose
    // where _pose is the canonical link's world pose
    this->parentEntity->worldPose = (-this->initialRelativePose) + _pose;

    this->parentEntity->worldPose.Correct();

    if (_notify)
      this->parentEntity->UpdatePhysicsPose(false);

    if (_publish)
      this->parentEntity->PublishPose();

    // Tell collisions that their current world pose is dirty (needs
    // updating). We set a dirty flag instead of directly updating the
    // value to improve performance.
    for (Base_V::iterator iterC = this->children.begin();
        iterC != this->children.end(); ++iterC)
    {
      if ((*iterC)->HasType(COLLISION))
      {
        CollisionPtr entityC = boost::static_pointer_cast<Collision>(*iterC);
        entityC->SetWorldPoseDirty();
      }
    }
  }
  else
    gzerr << "SWP for CB[" << this->GetName() << "] but parent["
      << this->parentEntity->GetName() << "] is not a MODEL!\n";
}

//////////////////////////////////////////////////
void Entity::SetWorldPoseDefault(const math::Pose &_pose, bool _notify,
        bool /*_publish*/)
{
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify)
    this->UpdatePhysicsPose(true);
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
void Entity::SetWorldPose(const math::Pose &_pose, bool _notify, bool _publish)
{
  {
    boost::mutex::scoped_lock lock(*this->GetWorld()->GetSetWorldPoseMutex());
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
        coll->worldPose.pos = this->worldPose.pos +
          this->worldPose.rot.RotateVector(coll->initialRelativePose.pos);
        coll->worldPose.rot = this->worldPose.rot *
          coll->initialRelativePose.rot;
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
  if (this->requestPub)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete",
        this->GetScopedName());
    this->requestPub->Publish(*msg, true);
  }

  this->parentEntity.reset();
  Base::Fini();

  this->connections.clear();
  this->node->Fini();
}

//////////////////////////////////////////////////
void Entity::Reset()
{
  if (this->HasType(Base::MODEL))
    this->SetWorldPose(this->initialRelativePose);
  else
    this->SetRelativePose(this->initialRelativePose);
}

//////////////////////////////////////////////////
void Entity::UpdateParameters(sdf::ElementPtr _sdf)
{
  Base::UpdateParameters(_sdf);

  math::Pose parentPose;
  if (this->parent && this->parentEntity)
    parentPose = this->parentEntity->worldPose;

  math::Pose newPose = _sdf->Get<math::Pose>("pose");
  if (newPose != this->GetRelativePose())
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

  math::Pose offset;
  offset.pos = kf.Translation();
  offset.rot = kf.Rotation();

  this->SetWorldPose(offset);
  this->prevAnimationTime = _info.simTime;

  if (this->animation->GetLength() <= this->animation->GetTime())
  {
    event::Events::DisconnectWorldUpdateBegin(this->animationConnection);
    this->animationConnection.reset();
    if (this->onAnimationComplete)
    {
      this->onAnimationComplete();
    }
  }
}

//////////////////////////////////////////////////
const math::Pose &Entity::GetDirtyPose() const
{
  return this->dirtyPose;
}

//////////////////////////////////////////////////
math::Box Entity::GetCollisionBoundingBox() const
{
  BasePtr base = boost::const_pointer_cast<Base>(shared_from_this()); return
  this->GetCollisionBoundingBoxHelper(base);
}

//////////////////////////////////////////////////
math::Box Entity::GetCollisionBoundingBoxHelper(BasePtr _base) const
{
  if (_base->HasType(COLLISION))
    return boost::dynamic_pointer_cast<Collision>(_base)->GetBoundingBox();

  math::Box box;

  for (unsigned int i = 0; i < _base->GetChildCount(); i++)
  {
    box += this->GetCollisionBoundingBoxHelper(_base->GetChild(i));
  }

  return box;
}

//////////////////////////////////////////////////
void Entity::PlaceOnEntity(const std::string &_entityName)
{
  EntityPtr onEntity = this->GetWorld()->GetEntity(_entityName);
  math::Box box = this->GetCollisionBoundingBox();
  math::Box onBox = onEntity->GetCollisionBoundingBox();

  math::Pose p = onEntity->GetWorldPose();
  p.pos.z = onBox.max.z + box.GetZLength()*0.5;
  this->SetWorldPose(p);
}

//////////////////////////////////////////////////
void Entity::GetNearestEntityBelow(double &_distBelow,
                                   std::string &_entityName)
{
  this->GetWorld()->GetPhysicsEngine()->InitForThread();
  RayShapePtr rayShape = boost::dynamic_pointer_cast<RayShape>(
    this->GetWorld()->GetPhysicsEngine()->CreateShape("ray", CollisionPtr()));

  math::Box box = this->GetCollisionBoundingBox();
  math::Vector3 start = this->GetWorldPose().pos;
  math::Vector3 end = start;
  start.z = box.min.z - 0.00001;
  end.z -= 1000;
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
    math::Pose p = this->GetWorldPose();
    p.pos.z -= dist;
    this->SetWorldPose(p);
  }
}
