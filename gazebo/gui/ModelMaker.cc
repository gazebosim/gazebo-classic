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

#include <sstream>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelMaker.hh"
#include "gazebo/gui/ModelMakerPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelMaker::ModelMaker() : dataPtr(new ModelMakerPrivate)
{
  this->dataPtr->clone = false;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->makerPub =
      this->dataPtr->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
ModelMaker::~ModelMaker()
{
  this->dataPtr->makerPub.reset();
  this->dataPtr->node->Fini();
  this->dataPtr->node.reset();
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromModel(const std::string &_modelName)
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (!scene)
    return false;

  if (this->dataPtr->modelVisual)
  {
    scene->RemoveVisual(this->dataPtr->modelVisual);
    this->dataPtr->modelVisual.reset();
    this->dataPtr->visuals.clear();
  }

  rendering::VisualPtr vis = scene->GetVisual(_modelName);
  if (!vis)
  {
    gzerr << "Model: '" << _modelName << "' does not exist." << std::endl;
    return false;
  }

  this->dataPtr->modelVisual = vis->Clone(
      _modelName + "_clone_tmp", scene->WorldVisual());

  if (!this->dataPtr->modelVisual)
  {
    gzerr << "Unable to clone\n";
    return false;
  }
  this->dataPtr->clone = true;
  return true;
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromFile(const std::string &_filename)
{
  this->dataPtr->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (this->dataPtr->modelVisual)
  {
    scene->RemoveVisual(this->dataPtr->modelVisual);
    this->dataPtr->modelVisual.reset();
  }

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->dataPtr->modelSDF);

  if (!sdf::readFile(_filename, this->dataPtr->modelSDF))
  {
    gzerr << "Unable to load file[" << _filename << "]\n";
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::InitSimpleShape(SimpleShapes _shape)
{
  this->dataPtr->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (!scene)
    return false;

  if (this->dataPtr->modelVisual)
  {
    scene->RemoveVisual(this->dataPtr->modelVisual);
    this->dataPtr->modelVisual.reset();
  }

  // Desired name (the server will append a number to generate a unique name
  // in case of overlap.
  std::string modelName;
  if (_shape == BOX)
    modelName = "unit_box";
  else if (_shape == SPHERE)
    modelName = "unit_sphere";
  else if (_shape == CYLINDER)
    modelName = "unit_cylinder";

  // Model message
  msgs::Model model;
  model.set_name(modelName);
  msgs::Set(model.mutable_pose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  if (_shape == BOX)
    msgs::AddBoxLink(model, 1.0, ignition::math::Vector3d::One);
  else if (_shape == SPHERE)
    msgs::AddSphereLink(model, 1.0, 0.5);
  else if (_shape == CYLINDER)
    msgs::AddCylinderLink(model, 1.0, 0.5, 1.0);
  model.mutable_link(0)->set_name("link");

  // Model SDF
  std::string modelString = "<sdf version='" + std::string(SDF_VERSION) + "'>"
       + msgs::ModelToSDF(model)->ToString("") + "</sdf>";

  this->dataPtr->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->dataPtr->modelSDF);

  if (!sdf::readString(modelString, this->dataPtr->modelSDF))
  {
    gzerr << "Unable to load SDF [" << modelString << "]" << std::endl;
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::Init()
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (!scene)
    return false;

  // Load the world file
  std::string modelName;
  ignition::math::Pose3d modelPose, linkPose, visualPose;
  sdf::ElementPtr modelElem;

  if (this->dataPtr->modelSDF->Root()->HasElement("model"))
    modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");
  else if (this->dataPtr->modelSDF->Root()->HasElement("light"))
    modelElem = this->dataPtr->modelSDF->Root()->GetElement("light");
  else
  {
    gzerr << "No model or light in SDF\n";
    return false;
  }

  if (modelElem->HasElement("pose"))
    modelPose = modelElem->Get<ignition::math::Pose3d>("pose");

  modelName = modelElem->Get<std::string>("name");

  this->dataPtr->modelVisual.reset(new rendering::Visual(
      this->dataPtr->node->GetTopicNamespace() + "::" + modelName,
      scene->WorldVisual()));
  this->dataPtr->modelVisual->Load();
  this->dataPtr->modelVisual->SetPose(modelPose);

  modelName = this->dataPtr->modelVisual->GetName();
  modelElem->GetAttribute("name")->Set(modelName);

  if (modelElem->GetName() == "model")
  {
    this->CreateModelFromSDF(modelElem);
  }
  else if (modelElem->GetName() == "light")
  {
    this->dataPtr->modelVisual->AttachMesh("unit_sphere");
  }
  else
  {
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void ModelMaker::CreateModelFromSDF(sdf::ElementPtr _modelElem)
{
  ignition::math::Pose3d linkPose, visualPose;
  std::list<std::pair<sdf::ElementPtr, rendering::VisualPtr> > modelElemList;

  std::pair<sdf::ElementPtr, rendering::VisualPtr> pair(
      _modelElem, this->dataPtr->modelVisual);
  modelElemList.push_back(pair);

  while (!modelElemList.empty())
  {
    sdf::ElementPtr modelElem = modelElemList.front().first;
    rendering::VisualPtr modelVis = modelElemList.front().second;
    modelElemList.pop_front();

    std::string modelName = modelVis->GetName();

    // create model
    sdf::ElementPtr linkElem;
    if (modelElem->HasElement("link"))
      linkElem = modelElem->GetElement("link");

    try
    {
      while (linkElem)
      {
        std::string linkName = linkElem->Get<std::string>("name");
        if (linkElem->HasElement("pose"))
          linkPose = linkElem->Get<ignition::math::Pose3d>("pose");
        else
          linkPose.Set(0, 0, 0, 0, 0, 0);

        rendering::VisualPtr linkVisual(new rendering::Visual(modelName + "::" +
              linkName, modelVis));
        linkVisual->Load();
        linkVisual->SetPose(linkPose);
        this->dataPtr->visuals.push_back(linkVisual);

        int visualIndex = 0;
        sdf::ElementPtr visualElem;

        if (linkElem->HasElement("visual"))
          visualElem = linkElem->GetElement("visual");

        while (visualElem)
        {
          if (visualElem->HasElement("pose"))
            visualPose = visualElem->Get<ignition::math::Pose3d>("pose");
          else
            visualPose.Set(0, 0, 0, 0, 0, 0);

          std::ostringstream visualName;
          visualName << modelName << "::" << linkName << "::Visual_"
            << visualIndex++;
          rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
                linkVisual));

          visVisual->Load(visualElem);
          visVisual->SetPose(visualPose);
          this->dataPtr->visuals.push_back(visVisual);

          visualElem = visualElem->GetNextElement("visual");
        }

        linkElem = linkElem->GetNextElement("link");
      }
    }
    catch(common::Exception &_e)
    {
      this->Stop();
    }

    // append other model elems to the list
    if (modelElem->HasElement("model"))
    {
      sdf::ElementPtr childElem = modelElem->GetElement("model");
      while (childElem)
      {
        rendering::VisualPtr childVis;
        std::string childName = childElem->Get<std::string>("name");
        childVis.reset(new rendering::Visual(modelName + "::" + childName,
            modelVis));
        childVis->Load();
        this->dataPtr->visuals.push_back(childVis);

        ignition::math::Pose3d childPose;
        if (childElem->HasElement("pose"))
          childPose = childElem->Get<ignition::math::Pose3d>("pose");
        childVis->SetPose(childPose);

        std::pair<sdf::ElementPtr, rendering::VisualPtr> childPair(
            childElem, childVis);
        modelElemList.push_back(childPair);

        childElem = childElem->GetNextElement("model");
      }
    }
  }
}

/////////////////////////////////////////////////
void ModelMaker::Stop()
{
  // Remove the temporary visual from the scene
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (scene)
  {
    for (auto vis : this->dataPtr->visuals)
    {
      auto v = vis.lock();
      if (v)
        scene->RemoveVisual(v);
    }
    scene->RemoveVisual(this->dataPtr->modelVisual);
  }
  this->dataPtr->modelVisual.reset();
  this->dataPtr->modelSDF.reset();

  EntityMaker::Stop();
}

/////////////////////////////////////////////////
void ModelMaker::CreateTheEntity()
{
  msgs::Factory msg;
  if (!this->dataPtr->clone)
  {
    sdf::ElementPtr modelElem;
    if (this->dataPtr->modelSDF->Root()->HasElement("model"))
    {
      modelElem = this->dataPtr->modelSDF->Root()->GetElement("model");
    }
    else
    {
      gzerr << "Couldn't find model element in sdf, won't create entity."
            << std::endl;
      return;
    }

    // The server will generate a unique name in case of name collision
    std::string modelName = modelElem->Get<std::string>("name");

    // Remove the topic namespace from the model name. This will get re-inserted
    // by the World automatically
    modelName.erase(0, this->dataPtr->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelName);
    modelElem->GetElement("pose")->Set(
        this->dataPtr->modelVisual->GetWorldPose().Ign());

    // Spawn the model in the physics server
    msg.set_sdf(this->dataPtr->modelSDF->ToString());
  }
  else
  {
    msgs::Set(msg.mutable_pose(),
        this->dataPtr->modelVisual->GetWorldPose().Ign());
    msg.set_clone_model_name(this->dataPtr->modelVisual->GetName().substr(0,
          this->dataPtr->modelVisual->GetName().find("_clone_tmp")));
  }

  this->dataPtr->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelMaker::EntityPosition() const
{
  return this->dataPtr->modelVisual->GetWorldPose().pos.Ign();
}

/////////////////////////////////////////////////
void ModelMaker::SetEntityPosition(const ignition::math::Vector3d &_pos)
{
  this->dataPtr->modelVisual->SetWorldPosition(_pos);
}
