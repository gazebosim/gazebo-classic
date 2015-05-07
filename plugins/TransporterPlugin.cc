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

#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include "plugins/TransporterPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(TransporterPlugin)

/////////////////////////////////////////////////
TransporterPlugin::TransporterPlugin()
{
}

/////////////////////////////////////////////////
TransporterPlugin::~TransporterPlugin()
{
}

/////////////////////////////////////////////////
void TransporterPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "TransporterPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "TransporterPlugin sdf pointer is NULL");
  this->world = _world;
  this->sdf = _sdf;

  sdf::ElementPtr padElem = _sdf->GetElement("pad");
  while (padElem)
  {
    TransporterPlugin::Pad *pad = new TransporterPlugin::Pad;

    pad->name = padElem->Get<std::string>("name");
    pad->dest = padElem->Get<std::string>("destination");

    if (padElem->HasElement("activation"))
    {
      pad->autoActivation =
        padElem->Get<std::string>("activation") == "auto" ? true : false;
    }
    else
    {
      pad->autoActivation = true;
    }

    sdf::ElementPtr outElem = padElem->GetElement("outgoing");
    pad->outgoingPose = outElem->Get<math::Pose>("pose");
    pad->outgoingBox = outElem->Get<math::Vector3>("box");

    sdf::ElementPtr inElem = padElem->GetElement("incoming");
    pad->incomingPose = inElem->Get<math::Pose>("pose");
    pad->incomingBox = inElem->Get<math::Vector3>("box");

    this->pads[pad->name] = pad;

    padElem = padElem->GetNextElement("pad");
  }

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TransporterPlugin::Update, this));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_world->GetName());

  this->activationSub = this->node->Subscribe(
      this->sdf->Get<std::string>("activation_topic"),
      &TransporterPlugin::OnActivation, this);
}

/////////////////////////////////////////////////
void TransporterPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  std::map<std::string, Pad*>::iterator iter = this->pads.find(_msg->data());

  {
    if (iter != this->pads.end())
    {
      boost::mutex::scoped_lock lock(this->padMutex);
      std::cout << "Activated[" << _msg->data() << "]\n";
      iter->second->activated = true;
    }
  }
}

/////////////////////////////////////////////////
void TransporterPlugin::Update()
{
  physics::Model_V models = this->world->GetModels();

  boost::mutex::scoped_lock lock(this->padMutex);

  // Process each model.
  for (physics::Model_V::iterator iter = models.begin();
       iter != models.end(); ++iter)
  {
    // Skip models that are static
    if ((*iter)->IsStatic())
      continue;

    math::Pose modelPose = (*iter)->GetWorldPose();
    for (std::map<std::string, Pad*>::iterator padIter = this->pads.begin();
         padIter != this->pads.end(); ++padIter)
    {
      math::Vector3 min = padIter->second->outgoingPose.pos -
                          padIter->second->outgoingBox / 2.0;

      math::Vector3 max = padIter->second->outgoingPose.pos +
                          padIter->second->outgoingBox / 2.0;

      if (modelPose.pos.x > min.x && modelPose.pos.x < max.x &&
          modelPose.pos.y > min.y && modelPose.pos.y < max.y &&
          modelPose.pos.z > min.z && modelPose.pos.z < max.z)
      {
        std::map<std::string, Pad*>::iterator destIter =
          this->pads.find(padIter->second->dest);

        /*std::cout << "Inside Pad[" << padIter->first << "] Pose["
                  << modelPose.pos << "] Dest["
                  << padIter->second->dest << "]\n";
                  */

        if (destIter != this->pads.end() &&
            (padIter->second->autoActivation || padIter->second->activated))
        {
          /*physics::ModelPtr destModel = this->world->GetModelBelowPoint(
              destIter->second->incomingPose.pos);

          math::Pose destPose = destIter->second->incomingPose;
          (*iter)->SetWorldPose(destPose);

          double dz = (*iter)->GetBoundingBox().min.z -
                      destModel->GetBoundingBox().max.z;
          destPose.pos.z -= dz;
          (*iter)->SetWorldPose(destPose);
          */
          math::Pose destPose = destIter->second->incomingPose;
          (*iter)->SetWorldPose(destPose);

          padIter->second->activated = false;
        }
      }
    }
  }
}
