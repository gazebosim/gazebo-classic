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

#include <gazebo/common/Console.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "ARATPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ARATPlugin)

/////////////////////////////////////////////////
ARATPlugin::ARATPlugin()
{
}

/////////////////////////////////////////////////
void ARATPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->sdf = _sdf;

  // Fill this->objects with initial poses of named models
  {
    const std::string elemName = "modelName";
    if (this->sdf->HasElement(elemName))
    {
      sdf::ElementPtr elem = this->sdf->GetElement(elemName);
      while (elem)
      {
        // Add names to map
        std::string modelName = elem->Get<std::string>();
        ObjectPtr object(new Object);
        object->model = world->GetModel(modelName);
        object->pose = object->model->GetWorldPose();
        this->objects[modelName] = object;

        elem = elem->GetNextElement(elemName);
      }
    }
  }

  // Get task information
  {
    const std::string elemName = "task";
    if (this->sdf->HasElement(elemName))
    {
      sdf::ElementPtr elem = this->sdf->GetElement(elemName);
      while (elem)
      {
        // Read task name attribute
        if (!elem->HasAttribute("name"))
        {
          gzerr << "task element missing name attribute" << std::endl;
          continue;
        }
        std::string taskName = elem->Get<std::string>("name");

        // Read pose elements into Pose_M
        Pose_M poses;
        if (elem->HasElement("pose"))
        {
          sdf::ElementPtr poseElem = elem->GetElement("pose");
          while (poseElem)
          {
            // Read pose name attribute
            if (!poseElem->HasAttribute("name"))
            {
              gzerr << "pose element missing name attribute" << std::endl;
              continue;
            }
            std::string poseName = poseElem->Get<std::string>("name");
            poses[poseName] = poseElem->Get<math::Pose>();

            poseElem = poseElem->GetNextElement("pose");
          }
        }
        this->tasks[taskName] = poses;
        gzdbg << "Loaded task ["
              << taskName
              << "] with "
              << poses.size()
              << " poses"
              << std::endl;

        elem = elem->GetNextElement(elemName);
      }
    }
  }
}

/////////////////////////////////////////////////
void ARATPlugin::Init()
{
  // Set default task
  gzdbg << "Set wood_blocks task" << std::endl;
  this->SetTask("wood_blocks");
}

/////////////////////////////////////////////////
bool ARATPlugin::SetTask(const std::string &_task)
{
  if (this->tasks.find(_task) == this->tasks.end())
  {
    gzerr << "Cannot SetTask ["
          << _task
          << "], unrecognized task name"
          << std::endl;
    return false;
  }

  Pose_M task = this->tasks[_task];

  for (Object_M::iterator iter = this->objects.begin();
        iter != this->objects.end(); ++iter)
  {
    physics::ModelPtr model = iter->second->model;
    math::Pose pose = iter->second->pose;
    Pose_M::iterator poseIter = task.find(iter->first);
    if (poseIter != task.end())
    {
      // object name found in task
      pose = poseIter->second;
    }
    model->SetWorldPose(pose);
    model->ResetPhysicsStates();
  }
  return true;
}

