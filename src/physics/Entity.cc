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

#include "common/Console.hh"

#include "transport/Publisher.hh"
#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "physics/Geom.hh"
#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/Link.hh"
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

  this->visualMsg->set_mesh_type( msgs::Visual::UNKNOWN );

  if (this->parent && this->parent->HasType(ENTITY))
  {
    this->parentEntity = boost::shared_dynamic_cast<Entity>(this->parent);
    this->SetStatic( this->parentEntity->IsStatic() );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Entity::~Entity()
{
  Base_V::iterator iter;

  // TODO: put this back in
  //this->GetWorld()->GetPhysicsEngine()->RemoveEntity(this);

  // Tell all renderers that I'm gone
  this->visualMsg->set_action( msgs::Visual::DELETE );
  this->visPub->Publish(*this->visualMsg);
  delete this->visualMsg;
  this->visualMsg = NULL;

  delete this->poseMsg;
  this->poseMsg = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Load
void Entity::Load(sdf::ElementPtr &_sdf)
{
  Base::Load(_sdf);
  this->node->Init(this->GetWorld()->GetName());
  this->posePub = this->node->Advertise<msgs::Pose>("~/pose", 10);
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

  this->visualMsg->mutable_header()->set_str_id(this->GetCompleteScopedName());

  if (_sdf->HasElement("origin"))
  {
    if (this->parent && this->parentEntity)
      this->worldPose = _sdf->GetElement("origin")->GetValuePose("pose") +
                        this->parentEntity->worldPose;
    else
      this->worldPose = _sdf->GetElement("origin")->GetValuePose("pose");
    this->initialRelativePose = _sdf->GetElement("origin")->GetValuePose("pose");
  }

  if (this->parent)
    this->visualMsg->set_parent_id( this->parent->GetCompleteScopedName() );

  this->visPub->Publish(*this->visualMsg);

  msgs::Init( *this->poseMsg, this->GetCompleteScopedName() );
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

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
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

// Am I a canonical Link for my Model parent?
bool Entity::IsCanonicalLink() const
{
  return this->isCanonicalLink;
}

void Entity::PublishPose()
{
  msgs::Set( this->poseMsg, this->GetRelativePose());
  this->posePub->Publish( *this->poseMsg);
}

////////////////////////////////////////////////////////////////////////////////
// Get the pose of the entity relative to its parent
math::Pose Entity::GetRelativePose() const
{
  if (this->IsCanonicalLink())
  {
    return this->initialRelativePose; // this should never change for canonical
  }
  else if (this->parent && this->parentEntity)
  {
    return this->worldPose - this->parentEntity->GetWorldPose();
  }
  else
    return this->worldPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the entity relative to its parent
void Entity::SetRelativePose(const math::Pose &pose, bool notify)
{
  if (this->parent && this->parentEntity)
    this->SetWorldPose(pose + this->parentEntity->GetWorldPose(), notify);
  else
    this->SetWorldPose(pose, notify);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the absolute pose of the entity
math::Pose Entity::GetWorldPose() const
{
  /* debugging
  if (this->HasType(MODEL))
  {
    //gzerr << "GWP [" << this->GetName() << "]\n";
    // simply check that WP = CWP - CIRP for assurance
    const Model* model = dynamic_cast<const Model*>(this);
    LinkPtr link = model->GetLink();
    if (!link)
      gzerr << "  No CB for [" << this->GetName() << "] Init?\n";
    else
    {

      math::Pose model_abs_pose1 = link->worldPose + link->initialRelativePose.GetInverse();
      math::Pose model_abs_pose2;
      math::Pose cb_rel_pose = this->GetRelativePose();
      // anti-rotate cb-abs by cb-rel = model-abs
      model_abs_pose2.rot = link->worldPose.rot * link->initialRelativePose.rot.GetInverse();
      // rotate cb-rel pos by cb-rel rot to get pos offset
      //Vector3 pos_offset = cb->GetRelativePose().rot.GetInverse() * cb->GetRelativePose().pos;
      // finally, model-abs pos is cb-abs pos - pos_offset
      //model_abs_pose2.pos = cb->GetWorldPose().pos - pos_offset;
      model_abs_pose2.pos = link->worldPose.pos - model_abs_pose2.rot * link->initialRelativePose.pos;
      //model_abs_pose2.pos = pose.pos - this->GetRelativePose().pos;
      // set abs pose of parent model without propagating
      // changes to children

      if (this->worldPose != model_abs_pose1 || this->worldPose != model_abs_pose2)
      {
        gzerr << "        GWP[ " << this->worldPose << "]\n";
        gzerr << "  *CWP-CIRP[ " << model_abs_pose1 << "]\n";
        gzerr << "  *CWP-CIRP[ " << model_abs_pose2 << "]\n";
      }
    }
  }
  */

  return this->worldPose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs twist of the entity
void Entity::SetWorldTwist(const math::Vector3 &linear, const math::Vector3 &angular, bool updateChildren)
{
  if (this->HasType(BODY) || this->HasType(MODEL))
  {
    if (this->HasType(BODY))
    {
      Link* link = dynamic_cast<Link*>(this);
      link->SetLinearVel(linear);
      link->SetAngularVel(angular);
    }
    if (updateChildren)
    {
      // force an update of all children
      for  (Base_V::iterator iter = this->children.begin();
            iter != this->children.end(); iter++)
      {
        if ((*iter)->HasType(ENTITY))
        {
          EntityPtr entity = boost::shared_static_cast<Entity>(*iter);
          entity->SetWorldTwist(linear,angular,updateChildren);
        }
      }
    }
  }
  //else
  //  gzdbg << "Setting Twist of a non-ODE body [" << this->GetName() << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set the abs pose of the entity
//   The entity stores an initialRelativePose and dynamic worldPose
//   When calling SetWroldPose (SWP) or SetRelativePose on an entity
//   that is a Model (M), Canonical Body (CB) or Body (B), different
//   considerations need to be taken.
void Entity::SetWorldPose(const math::Pose &pose, bool notify)
{
  if (this->HasType(MODEL))
  {
    this->GetWorld()->modelWorldPoseUpdateMutex->lock();

    //gzerr << "SWP [" << this->GetName() << "]\n";
    //printf("\nSWP Model [%s]\n",this->GetName().c_str());

    math::Pose oldModelWorldPose = this->worldPose;

    // initialization: (no children?) set own worldPose
    this->worldPose = pose;
    this->worldPose.Correct();
    if (notify) this->UpdatePhysicsPose(false); // (OnPoseChange uses GetWorldPose)

    //
    // user deliberate setting: lock and update all children's wp
    //

    // force an update of all children
    // update all children pose, moving them with the model.
    for  (Base_V::iterator iter = this->children.begin();
          iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(ENTITY))
      {
        EntityPtr entity = boost::shared_static_cast<Entity>(*iter);
        if (entity->IsCanonicalLink())
          entity->worldPose = (entity->initialRelativePose + pose);
        else
          entity->worldPose = ((entity->worldPose - oldModelWorldPose) + pose);
        if (notify) entity->UpdatePhysicsPose(false);
        entity->PublishPose();
        //printf("SWP Model Body [%s]\t",(*iter)->GetName().c_str());
      }
    }

    //printf("\nSWP Model [%s] DONE\n\n",this->GetName().c_str());
    this->GetWorld()->modelWorldPoseUpdateMutex->unlock();
  }
  else if (this->IsCanonicalLink())
  {
    this->GetWorld()->modelWorldPoseUpdateMutex->lock();
    //printf("c[%s]",this->GetName().c_str());
    this->worldPose = pose;
    this->worldPose.Correct();
    if (notify) this->UpdatePhysicsPose(true);

    // also update parent model's pose
    if (this->parentEntity->HasType(MODEL))
    {
      this->parentEntity->worldPose = pose - this->initialRelativePose;
      this->parentEntity->worldPose.Correct();
      if (notify) this->parentEntity->UpdatePhysicsPose(false);
      this->parentEntity->PublishPose();
    }
    else
      gzerr << "SWP for CB[" << this->GetName() << "] but parent["
            << this->parentEntity->GetName() << "] is not a MODEL!\n";
    //printf("C[%s]",this->GetName().c_str());
    this->GetWorld()->modelWorldPoseUpdateMutex->unlock();
  }
  else
  {
    this->GetWorld()->modelWorldPoseUpdateMutex->lock();
    //printf("b[%s]",this->GetName().c_str());
    this->worldPose = pose;
    this->worldPose.Correct();
    if (notify) this->UpdatePhysicsPose(true);
    //printf("B[%s]",this->GetName().c_str());
    this->GetWorld()->modelWorldPoseUpdateMutex->unlock();
  }

  this->PublishPose();
}

////////////////////////////////////////////////////////////////////////////////
// Handle a change of pose
void Entity::UpdatePhysicsPose(bool update_children)
{
  this->OnPoseChange();

  if (update_children)
  {
    for  (Base_V::iterator iter = this->children.begin();
          iter != this->children.end(); iter++)
    {
      if ((*iter)->HasType(ENTITY))
        boost::shared_static_cast<Entity>(*iter)->OnPoseChange();
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
