/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "plugins/ArrangePlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ArrangePlugin)

/////////////////////////////////////////////////
ArrangePlugin::ArrangePlugin()
{
}

/////////////////////////////////////////////////
ArrangePlugin::~ArrangePlugin()
{
}

/////////////////////////////////////////////////
void ArrangePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "ArrangePlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "ArrangePlugin sdf pointer is NULL");
  this->world = _world;
  this->sdf = _sdf;

  // Fill this->objects with initial poses of named models
  {
    const std::string elemName = "model_name";
    if (this->sdf->HasElement(elemName))
    {
      sdf::ElementPtr elem = this->sdf->GetElement(elemName);
      while (elem)
      {
        // Add names to map
        std::string modelName = elem->Get<std::string>();
        physics::ModelPtr model = world->GetModel(modelName);
        if (model)
        {
          ObjectPtr object(new Object);
          object->model = model;
          object->pose = model->GetWorldPose();
          this->objects[modelName] = object;
        }
        else
        {
          gzerr << "Unable to get model ["
                << modelName
                << "], skipping"
                << std::endl;
        }

        elem = elem->GetNextElement(elemName);
      }
    }
  }

  // Get name of topic to listen on
  {
    const std::string elemName = "topic_name";
    if (this->sdf->HasElement(elemName))
    {
      this->eventTopicName = this->sdf->Get<std::string>(elemName);
    }
  }

  // Get initial arrangement name
  {
    const std::string elemName = "initial_arrangement";
    if (this->sdf->HasElement(elemName))
    {
      this->initialArrangementName = this->sdf->Get<std::string>(elemName);
    }
  }

  // Get arrangement information
  {
    const std::string elemName = "arrangement";
    if (this->sdf->HasElement(elemName))
    {
      sdf::ElementPtr elem = this->sdf->GetElement(elemName);
      while (elem)
      {
        // Read arrangement name attribute
        if (!elem->HasAttribute("name"))
        {
          gzerr << "arrangement element missing name attribute" << std::endl;
          continue;
        }
        std::string arrangementName = elem->Get<std::string>("name");
        if (this->initialArrangementName.empty())
        {
          this->initialArrangementName = arrangementName;
        }

        // Read pose elements into Pose_M
        Pose_M poses;
        if (elem->HasElement("pose"))
        {
          sdf::ElementPtr poseElem = elem->GetElement("pose");
          while (poseElem)
          {
            // Read pose model attribute
            if (!poseElem->HasAttribute("model"))
            {
              gzerr << "In arrangement ["
                    << arrangementName
                    << "], a pose element is missing the model attribute"
                    << std::endl;
              continue;
            }
            std::string poseName = poseElem->Get<std::string>("model");
            poses[poseName] = poseElem->Get<math::Pose>();

            poseElem = poseElem->GetNextElement("pose");
          }
        }
        this->arrangements[arrangementName] = poses;

        elem = elem->GetNextElement(elemName);
      }
    }
  }

  // Subscribe to the topic specified in the world file

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  sub = this->node->Subscribe(this->eventTopicName,
                              &ArrangePlugin::ArrangementCallback, this);
}

/////////////////////////////////////////////////
void ArrangePlugin::Init()
{
  // Set initial arrangement
  this->SetArrangement(this->initialArrangementName);
}

/////////////////////////////////////////////////
void ArrangePlugin::Reset()
{
  this->SetArrangement(this->currentArrangementName);
}

void ArrangePlugin::ArrangementCallback(ConstGzStringPtr &_msg)
{
  // Set arrangement to the requested id
  this->SetArrangement(_msg->data());
}

/////////////////////////////////////////////////
bool ArrangePlugin::SetArrangement(const std::string &_arrangement)
{
  if (this->arrangements.find(_arrangement) == this->arrangements.end())
  {
    gzerr << "Cannot SetArrangement ["
          << _arrangement
          << "], unrecognized arrangement name"
          << std::endl;
    return false;
  }

  this->currentArrangementName = _arrangement;
  Pose_M arrangement = this->arrangements[_arrangement];

  for (Object_M::iterator iter = this->objects.begin();
        iter != this->objects.end(); ++iter)
  {
    physics::ModelPtr model = iter->second->model;
    math::Pose pose;
    Pose_M::iterator poseIter = arrangement.find(iter->first);
    if (poseIter != arrangement.end())
    {
      // object name found in arrangement
      // use arrangement pose
      pose = poseIter->second;
    }
    else
    {
      // object name not found in arrangement
      // use initial pose
      pose = iter->second->pose;
    }
    model->SetWorldPose(pose);
    model->ResetPhysicsStates();
  }
  return true;
}

