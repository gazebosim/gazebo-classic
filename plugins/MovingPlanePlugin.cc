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

#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "plugins/MovingPlanePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MovingPlanePlugin)

/////////////////////////////////////////////////
MovingPlanePlugin::MovingPlanePlugin()
{
  this->distance = 0;
  this->direction = math::Vector3(1, 0, 0);
}

/////////////////////////////////////////////////
void MovingPlanePlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "MovingPlanePlugin _model pointer is NULL");
  this->model = _model;
  this->modelName = _model->GetName();
  this->sdf = _sdf;

  if (this->sdf->HasElement("direction"))
  {
    this->direction =
        this->sdf->Get<math::Vector3>("direction");
  }

  if (this->sdf->HasElement("distance"))
  {
    this->distance = this->sdf->Get<double>("distance");
  }


  if (this->sdf->HasElement("link"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link");
    while (elem)
    {
      physics::LinkPtr link = this->model->GetLink(elem->Get<std::string>());
      this->AddLink(link);
      elem = elem->GetNextElement("link");
    }
  }

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "MovingPlanePlugin world pointer is NULL");

  this->physics = this->world->GetPhysicsEngine();
  GZ_ASSERT(this->physics, "MovingPlanePlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "MovingPlanePlugin _sdf pointer is NULL");
}

/////////////////////////////////////////////////
void MovingPlanePlugin::Init()
{
  this->prevUpdateTime = this->model->GetWorld()->GetSimTime();

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MovingPlanePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void MovingPlanePlugin::AddLink(physics::LinkPtr _link)
{
  // setup pid for controlling link position
  std::vector<common::PID> *posPid = &this->posPids[_link->GetScopedName()];
  if (posPid->size() != 3)  // resize one for each axis
    posPid->resize(3);
  for (unsigned int i = 0; i < posPid->size(); ++i)
  {
    common::PID pid(500.0, 0, 10.0, 0, 0, 1e4, -1e4);
    (*posPid)[i] = pid;
  }

  // setup pid for controlling link orientation
  std::vector<common::PID> *rotPid = &this->rotPids[_link->GetScopedName()];
  if (rotPid->size() != 3)  // resize one for each axis
    rotPid->resize(3);
  for (unsigned int i = 0; i < rotPid->size(); ++i)
  {
    common::PID pid(50.0, 0, 1.0, 0, 0, 1e4, -1e4);
    (*rotPid)[i] = pid;
  }

  // initialize target pose as link's current pose
  this->targetPoses[_link->GetScopedName()] = _link->GetWorldPose();

  // initialize command wrench for link to zeros
  this->wrenches[_link->GetScopedName()] = physics::Wrench();

  this->links[_link->GetScopedName()] = _link;
}

/////////////////////////////////////////////////
void MovingPlanePlugin::OnUpdate()
{
  common::Time currTime = this->world->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;

  // Skip the update step if SimTime appears to have gone backward.
  // Negative update time wreaks havok on the integrators.
  // This happens when World::ResetTime is called.
  // TODO: fix this when World::ResetTime is improved
  if (stepTime > 0)
  {
    // compute wrench command updates
    std::map<std::string, math::Pose>::iterator piter;
    for (piter = this->targetPoses.begin(); piter != this->targetPoses.end();
      ++piter)
    {
      math::Pose currentPose = this->links[piter->first]->GetWorldPose();
      math::Pose targetPose = piter->second;

      // Step the target pose
      double speed = 0.001;

      targetPose += math::Pose(speed*this->direction,
          math::Quaternion(0, 0, 0));
      this->targetPoses[piter->first] = targetPose;

      math::Vector3 errorPos = currentPose.pos - targetPose.pos;

      math::Vector3 errorRot =
        (currentPose.rot * targetPose.rot.GetInverse()).GetAsEuler();

      physics::Wrench *commandWrench = &this->wrenches[piter->first];

      std::vector<common::PID> *posPid = &this->posPids[piter->first];
      commandWrench->force.x = (*posPid)[0].Update(errorPos.x, stepTime);
      commandWrench->force.y = (*posPid)[1].Update(errorPos.y, stepTime);
      commandWrench->force.z = (*posPid)[2].Update(errorPos.z, stepTime);
      std::vector<common::PID> *rotPid = &this->rotPids[piter->first];
      commandWrench->torque.x = (*rotPid)[0].Update(errorRot.x, stepTime);
      commandWrench->torque.y = (*rotPid)[1].Update(errorRot.y, stepTime);
      commandWrench->torque.z = (*rotPid)[2].Update(errorRot.z, stepTime);
    }

    // apply wrench commands to links
    if (!this->wrenches.empty())
    {
      std::map<std::string, physics::Wrench>::iterator iter;
      for (iter = this->wrenches.begin(); iter != this->wrenches.end(); ++iter)
      {
        physics::LinkPtr link = this->links[iter->first];
        if (link)
        {
          link->SetForce(iter->second.force);
          link->SetTorque(iter->second.torque);
        }
      }
    }

    // reset pose
    for (piter = this->targetPoses.begin(); piter != this->targetPoses.end();
      ++piter)
    {
      math::Pose currentPose = this->links[piter->first]->GetWorldPose();

      math::Vector3 dPos =
          (this->modelInitPose.pos + this->direction * this->distance) -
          currentPose.pos;
      if (this->direction.Dot(dPos.Normalize()) < 0.0)
      {
        math::Pose resetPose = math::Pose(this->modelInitPose.pos
          -this->distance * this->direction, math::Quaternion(0, 0, 0));
        this->links[piter->first]->SetWorldPose(resetPose);
        this->targetPoses[piter->first] = resetPose;
      }
    }
  }
  else
  {
    gzwarn << "time step <= 0 in MovingPlane Plugin, Reset simulation?" <<
        std::endl;
  }
}
