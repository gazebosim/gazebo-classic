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

#include "plugins/TransporterPluginPrivate.hh"
#include "plugins/TransporterPlugin.hh"

using namespace gazebo;

// Register the plugin
GZ_REGISTER_WORLD_PLUGIN(TransporterPlugin)

/////////////////////////////////////////////////
TransporterPlugin::TransporterPlugin()
  : dataPtr(new TransporterPluginPrivate)
{
}

/////////////////////////////////////////////////
TransporterPlugin::~TransporterPlugin()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void TransporterPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  bool hasManualActivation = false;

  GZ_ASSERT(_world, "TransporterPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "TransporterPlugin sdf pointer is NULL");

  this->dataPtr->world = _world;

  // Read each pad element
  sdf::ElementPtr padElem = _sdf->GetElement("pad");
  while (padElem)
  {
    // Create a new pad
    auto pad = std::make_shared<TransporterPluginPrivate::Pad>();

    // Set the pad's name and destination
    pad->name = padElem->Get<std::string>("name");
    pad->dest = padElem->Get<std::string>("destination");

    // Check that a pad does not exist
    if (this->dataPtr->pads.find(pad->name) != this->dataPtr->pads.end())
    {
      gzerr << "Transporter pad with name[" << pad->name << "] already exists."
        << "The duplicate pad will not be loaded\n";
      continue;
    }

    // Set the pad's activation type. The default is auto activation.
    if (padElem->HasElement("activation"))
    {
      pad->autoActivation = padElem->Get<std::string>("activation") == "auto";

      // Store that the user has at least one pad with manual activation.
      // This info is used to trigger a warning message below.
      if (!pad->autoActivation)
        hasManualActivation = true;
    }
    else
    {
      pad->autoActivation = true;
    }

    // Read the outgoing information
    sdf::ElementPtr outElem = padElem->GetElement("outgoing");
    pad->outgoingBox.min = outElem->Get<math::Vector3>("min");
    pad->outgoingBox.max = outElem->Get<math::Vector3>("max");

    // Read the incoming information
    sdf::ElementPtr inElem = padElem->GetElement("incoming");
    pad->incomingPose = inElem->Get<math::Pose>("pose");

    // Store the pad
    this->dataPtr->pads[pad->name] = pad;

    // Get the next pad element
    padElem = padElem->GetNextElement("pad");
  }

  // Connect to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TransporterPlugin::Update, this));

  // Listen on the activation topic, if present. This topic is used for
  // manual activation.
  if (_sdf->HasElement("activation_topic"))
  {
    // Create and initialize the node.
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->Init(_world->GetName());

    // Subscribe to the activation topic.
    this->dataPtr->activationSub = this->dataPtr->node->Subscribe(
        _sdf->Get<std::string>("activation_topic"),
        &TransporterPlugin::OnActivation, this);
  }
  else if (hasManualActivation)
  {
    gzerr << "Manual activation of a transporter pad has been requested, "
      << "but no <activation_topic> is present in the plugin's SDF.\n";
  }
}

/////////////////////////////////////////////////
void TransporterPlugin::OnActivation(ConstGzStringPtr &_msg)
{
  // Find the pad
  auto const &iter = this->dataPtr->pads.find(_msg->data());
  if (iter != this->dataPtr->pads.end())
  {
    // Activate the pad.
    std::lock_guard<std::mutex> lock(this->dataPtr->padMutex);
    iter->second->activated = true;
  }
  else
  {
    gzwarn << "Unknown transporter pad[" << _msg->data() << "]\n";
  }
}

/////////////////////////////////////////////////
void TransporterPlugin::Update()
{
  // Get all the models
  physics::Model_V models = this->dataPtr->world->GetModels();

  std::lock_guard<std::mutex> lock(this->dataPtr->padMutex);

  // Process each model.
  for (auto const &model : models)
  {
    // Skip models that are static
    if (model->IsStatic())
      continue;

    // Get the model's pose
    math::Pose modelPose = model->GetWorldPose();

    // Iterate over all pads
    for (auto const &padIter : this->dataPtr->pads)
    {
      // Check if the model is in the pad's outgoing box.
      if (padIter.second->outgoingBox.Contains(modelPose.pos))
      {
        // Get the destination pad
        auto const &destIter = this->dataPtr->pads.find(padIter.second->dest);

        // Make sure we can transport the model
        if (destIter != this->dataPtr->pads.end() &&
            (padIter.second->autoActivation || padIter.second->activated))
        {
          // Move the model
          model->SetWorldPose(destIter->second->incomingPose);

          // Deactivate the pad. This is used by manually activated pads.
          padIter.second->activated = false;
        }
      }
    }
  }
}
