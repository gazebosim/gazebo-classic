/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "msgs/msgs.h"

#include "common/Events.hh"
#include "common/Console.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"

#include "transport/Publisher.hh"
#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "physics/RayShape.hh"
#include "physics/Collision.hh"
#include "physics/Model.hh"
#include "physics/Collision.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Entity.hh"
#include <boost/thread/recursive_mutex.hpp>

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Entity::Entity(BasePtr parent)
  : Base(parent)
{
  this->isCanonicalLink = false;
  this->node = transport::NodePtr(new transport::Node());
  this->AddType(ENTITY);

  this->visualMsg = new msgs::Visual;
  this->poseMsg = new msgs::Pose;

  if (this->parent && this->parent->HasType(ENTITY))
  {
    this->parentEntity = boost::shared_dynamic_cast<Entity>(this->parent);
    this->SetStatic(this->parentEntity->IsStatic());
  }

  this->setWorldPoseFunc = &Entity::SetWorldPoseDefault;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  Base_V::iterator iter;

  // TODO: put this back in
  //this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  delete this->visualMsg;
  this->visualMsg = NULL;

  delete this->poseMsg;
  this->poseMsg = NULL;

  this->posePub.reset();
  this->visPub.reset();
  this->requestPub.reset();
  this->poseSub.reset();
  this->node.reset();
}

////////////////////////////////////////////////////////////////////////////////
/// Load
void Entity::Load(sdf::ElementPtr &_sdf)
{
  Base::Load(_sdf);
  this->node->Init(this->GetWorld()->GetName());
  this->posePub = this->node->Advertise<msgs::Pose>("~/pose/info", 10);

  this->poseSub = this->node->Subscribe("~/pose/modify", &Entity::OnPoseMsg, this);
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);
  this->requestPub = this->node->Advertise<msgs::Request>("~/request");

  this->visualMsg->set_name(this->GetCompleteScopedName());

  if (this->sdf->HasElement("origin"))
  {
    sdf::ElementPtr originElem = this->sdf->GetElement("origin");
    if (this->parent && this->parentEntity)
      this->worldPose = originElem->GetValuePose("pose") +
                        this->parentEntity->worldPose;
    else
      this->worldPose = originElem->GetValuePose("pose");
    this->initialRelativePose = originElem->GetValuePose("pose");

    originElem->GetAttribute("pose")->SetUpdateFunc( 
        boost::bind( &Entity::GetRelativePose, this ) );
  }

  if (this->parent)
    this->visualMsg->set_parent_name( this->parent->GetCompleteScopedName() );

  this->visPub->Publish(*this->visualMsg);

  this->poseMsg->set_name(this->GetCompleteScopedName());

  if (this->HasType(Base::MODEL))
    this->setWorldPoseFunc = &Entity::SetWorldPoseModel;
  else if (this->IsCanonicalLink())
    this->setWorldPoseFunc = &Entity::SetWorldPoseCanonicalLink;
  else
    this->setWorldPoseFunc = &Entity::SetWorldPoseDefault;
}

 
////////////////////////////////////////////////////////////////////////////////
// Set the name of the entity
void Entity::SetName(const std::string &name)
{
  // TODO: if an entitie's name is changed, then the old visual is never
  // removed. Should add in functionality to modify/update the visual
  Base::SetName(name);
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this entity is static: immovable
void Entity::SetStatic(const bool &s)
{
  Base_V::iterator iter;

  this->isStatic = s ;

  for (iter = this->children.begin(); iter != this->childrenEnd; iter++)
  {
    EntityPtr e = boost::shared_dynamic_cast<Entity>(*iter);
    if (e)
      e->SetStatic(s);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this entity is static
bool Entity::IsStatic() const
{
  return this->isStatic;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the initial pose
void Entity::SetInitialRelativePose(const math::Pose &p )
{
  this->initialRelativePose = p;
}


////////////////////////////////////////////////////////////////////////////////
/// Return the bounding box for the entity 
math::Box Entity::GetBoundingBox() const
{
  return math::Box(math::Vector3(0,0,0), math::Vector3(1,1,1));
}

// Set to true if this entity is a canonical link for a model.
void Entity::SetCanonicalLink(bool _value)
{
  this->isCanonicalLink = _value;
}

/// Set an animation for this entity
void Entity::SetAnimation( const common::PoseAnimationPtr &_anim )
{
  this->animationStartPose = this->worldPose;

  this->prevAnimationTime = this->world->GetSimTime();
  this->animation = _anim;
  this->connections.push_back(
     event::Events::ConnectWorldUpdateStart(
       boost::bind(&Entity::UpdateAnimation, this)));
}

void Entity::PublishPose()
{
  if (this->posePub && this->posePub->HasConnections())
  {
    math::Pose relativePose = this->GetRelativePose();
    if (relativePose != msgs::Convert(*this->poseMsg))
    {
      msgs::Set(this->poseMsg, this->worldPose);
      this->posePub->Publish(*this->poseMsg);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the entity relative to its parent
math::Pose Entity::GetRelativePose() const
{
  if (this->IsCanonicalLink())
  {
    // this should never change for canonical
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

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const math::Pose &_pose, bool notify)
{
  if (this->parent && this->parentEntity)
    this->SetWorldPose(_pose + this->parentEntity->GetWorldPose(), notify);
  else
    this->SetWorldPose(_pose, notify);
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs twist of the entity
void Entity::SetWorldTwist(const math::Vector3 &linear, const math::Vector3 &angular, bool updateChildren)
{
  if (this->HasType(LINK) || this->HasType(MODEL))
  {
    if (this->HasType(LINK))
    {
      Link* link = dynamic_cast<Link*>(this);
      link->SetLinearVel(linear);
      link->SetAngularVel(angular);
    }
    if (updateChildren)
    {
      // force an update of all children
      for  (Base_V::iterator iter = this->children.begin();
            iter != this->childrenEnd; iter++)
      {
        if ((*iter)->HasType(ENTITY))
        {
          EntityPtr entity = boost::shared_static_cast<Entity>(*iter);
          entity->SetWorldTwist(linear,angular,updateChildren);
        }
      }
    }
  }
}

void Entity::SetWorldPoseModel(const math::Pose &_pose, bool _notify)
{
  math::Pose oldModelWorldPose = this->worldPose;

  // initialization: (no children?) set own worldPose
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify) 
    this->UpdatePhysicsPose(false); // (OnPoseChange uses GetWorldPose)

  //
  // user deliberate setting: lock and update all children's wp
  //

  // force an update of all children
  // update all children pose, moving them with the model.
  for (Base_V::iterator iter = this->children.begin();
       iter != this->childrenEnd; iter++)
  {
    if ((*iter)->HasType(ENTITY))
    {
      Entity *entity = (Entity*)((*iter).get());

      if (entity->IsCanonicalLink())
      {
        entity->worldPose = (entity->initialRelativePose + _pose);
      }
      else
      {
        entity->worldPose = ((entity->worldPose - oldModelWorldPose) + _pose);
        entity->PublishPose();
      }

      if (_notify) 
        entity->UpdatePhysicsPose(false);
    }
  }
}

void Entity::SetWorldPoseCanonicalLink(const math::Pose &_pose, bool _notify)
{
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify) 
    this->UpdatePhysicsPose(true);

  // also update parent model's pose
  if (this->parentEntity->HasType(MODEL))
  {
    this->parentEntity->worldPose = _pose - this->initialRelativePose;
    this->parentEntity->worldPose.Correct();

    if (_notify) 
      this->parentEntity->UpdatePhysicsPose(false);

    this->parentEntity->PublishPose();
  }
  else
    gzerr << "SWP for CB[" << this->GetName() << "] but parent["
      << this->parentEntity->GetName() << "] is not a MODEL!\n";
}

void Entity::SetWorldPoseDefault(const math::Pose &_pose, bool _notify)
{
  this->worldPose = _pose;
  this->worldPose.Correct();

  if (_notify) 
    this->UpdatePhysicsPose(true);
}


////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
//   The entity stores an initialRelativePose and dynamic worldPose
//   When calling SetWroldPose (SWP) or SetRelativePose on an entity
//   that is a Model (M), Canonical Body (CB) or Body (B), different
//   considerations need to be taken.
// Below is a table that summarizes the current code.
//  +----------------------------------------------------------+
//  |     |     M          |  CB             |  B              |
//  |----------------------------------------------------------|
//  |SWP  | Lock           | Lock            | Set BWP         |
//  |     | Update MWP     | Set CBWP        |                 |
//  |     | SWP Children   | SWP M=CB-CBRP   |                 |
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
void Entity::SetWorldPose(const math::Pose &_pose, bool _notify)
{
  this->GetWorld()->modelWorldPoseUpdateMutex->lock();

  (*this.*setWorldPoseFunc)(_pose, _notify);

  this->GetWorld()->modelWorldPoseUpdateMutex->unlock();

  this->PublishPose();
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::UpdatePhysicsPose(bool _updateChildren)
{
  this->OnPoseChange();

  if (_updateChildren || this->IsStatic())
  {
    for  (Base_V::iterator iter = this->children.begin();
          iter != this->childrenEnd; iter++)
    {
      if ((*iter)->HasType(ENTITY))
      {
        ((Entity*)(*iter).get())->OnPoseChange();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the parent model, if one exists
ModelPtr Entity::GetParentModel() const
{
  BasePtr p = this->parent;

  while (p->GetParent() && p->GetParent()->HasType(MODEL))
    p = p->GetParent();

  return boost::shared_dynamic_cast<Model>(p);
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child collision entity, if one exists
CollisionPtr Entity::GetChildCollision(const std::string &_name)
{
  BasePtr base = this->GetByName(_name);
  if (base)
    return boost::shared_dynamic_cast<Collision>(base);
  
  return CollisionPtr();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child collision entity, if one exists
LinkPtr Entity::GetChildLink(const std::string &_name)
{
  BasePtr base = this->GetByName(_name);
  if (base)
    return boost::shared_dynamic_cast<Link>(base);
  
  return LinkPtr();
}

/// Called when a new pose message arrives
void Entity::OnPoseMsg( const boost::shared_ptr<msgs::Pose const> &_msg)
{
  if (_msg->name() == this->GetCompleteScopedName())
  {
    math::Pose p = msgs::Convert(*_msg);
    this->SetWorldPose( p );
  }
}

void Entity::Fini()
{
  msgs::Request *msg = msgs::CreateRequest("entity_delete", 
      this->GetCompleteScopedName());

  this->requestPub->Publish(*msg, true);

  this->parentEntity.reset();
  Base::Fini();

  this->connections.clear();
  this->node->Fini();
}

//// Update the parameters using new sdf values
void Entity::UpdateParameters( sdf::ElementPtr &_sdf )
{
  Base::UpdateParameters(_sdf);

  math::Pose parentPose;
  if (this->parent && this->parentEntity)
    parentPose = this->parentEntity->worldPose;

  math::Pose newPose = _sdf->GetElement("origin")->GetValuePose("pose");
  if (newPose != this->GetRelativePose())
  {
    this->SetRelativePose( newPose );
  }
}

void Entity::UpdateAnimation()
{
  common::PoseKeyFrame kf(0);

  this->animation->AddTime(
      (this->world->GetSimTime() - this->prevAnimationTime).Double());
  this->animation->GetInterpolatedKeyFrame(kf);
 
  math::Pose offset;
  offset.pos = kf.GetTranslate();
  offset.rot = kf.GetRotation();

  this->SetWorldPose(offset);
  this->prevAnimationTime = this->world->GetSimTime();
}


const math::Pose &Entity::GetDirtyPose() const
{
  return this->dirtyPose;
}

math::Box Entity::GetCollisionBoundingBox() const
{
  math::Box box;
  for (Base_V::const_iterator iter = this->children.begin();
       iter != this->children.end(); iter++)
  {
    box += this->GetCollisionBoundingBoxHelper(*iter);
  }

  return box;
}

math::Box Entity::GetCollisionBoundingBoxHelper(BasePtr _base) const
{
  if (_base->HasType(COLLISION))
    return boost::shared_dynamic_cast<Collision>(_base)->GetBoundingBox();

  math::Box box;

  for (unsigned int i=0; i < _base->GetChildCount(); i++)
  {
    box += this->GetCollisionBoundingBoxHelper(_base->GetChild(i));
  }

  return box;
}

void Entity::GetNearestEntityBelow(double &_distBelow,
                                   std::string &_entityName)
{
  RayShapePtr rayShape = boost::shared_dynamic_cast<RayShape>(
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
