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
#include "gazebo/gui/ModelMakerPrivate.hh"
#include "gazebo/gui/ModelMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelMaker::ModelMaker() : EntityMaker(*new ModelMakerPrivate)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->clone = false;
  dPtr->makerPub = dPtr->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
ModelMaker::~ModelMaker()
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->makerPub.reset();
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromModel(const std::string & _modelName)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (dPtr->modelVisual)
  {
    scene->RemoveVisual(dPtr->modelVisual);
    dPtr->modelVisual.reset();
  }

  rendering::VisualPtr vis = scene->GetVisual(_modelName);
  if (!vis)
  {
    gzerr << "Model: '" << _modelName << "' does not exist." << std::endl;
    return false;
  }

  dPtr->modelVisual = vis->Clone(
      _modelName + "_clone_tmp", scene->GetWorldVisual());

  if (!dPtr->modelVisual)
  {
    gzerr << "Unable to clone\n";
    return false;
  }
  dPtr->clone = true;
  return true;
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromSDFString(const std::string &_data)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (dPtr->modelVisual)
  {
    scene->RemoveVisual(dPtr->modelVisual);
    dPtr->modelVisual.reset();
  }

  dPtr->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", dPtr->modelSDF);
  sdf::readString(_data, dPtr->modelSDF);

  if (!sdf::readString(_data, dPtr->modelSDF))
  {
    gzerr << "Unable to load SDF from data\n";
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromFile(const std::string &_filename)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (dPtr->modelVisual)
  {
    scene->RemoveVisual(dPtr->modelVisual);
    dPtr->modelVisual.reset();
  }

  dPtr->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", dPtr->modelSDF);

  if (!sdf::readFile(_filename, dPtr->modelSDF))
  {
    gzerr << "Unable to load file[" << _filename << "]\n";
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::InitSimpleShape(SimpleShapes _shape)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (dPtr->modelVisual)
  {
    scene->RemoveVisual(dPtr->modelVisual);
    dPtr->modelVisual.reset();
  }

  // Unique name
  std::string prefix;
  if (_shape == BOX)
    prefix = "unit_box_";
  else if (_shape == SPHERE)
    prefix = "unit_sphere_";
  else if (_shape == CYLINDER)
    prefix = "unit_cylinder_";

  int counter = 0;
  std::ostringstream modelName;
  modelName << prefix << counter;
  while (scene->GetVisual(modelName.str()))
  {
    modelName.str("");
    modelName << prefix << counter;
    counter++;
  }

  // Model message
  msgs::Model model;
  model.set_name(modelName.str());
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

  dPtr->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", dPtr->modelSDF);

  if (!sdf::readString(modelString, dPtr->modelSDF))
  {
    gzerr << "Unable to load SDF [" << modelString << "]" << std::endl;
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::Init()
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  // Load the world file
  std::string modelName;
  ignition::math::Pose3d modelPose, linkPose, visualPose;
  sdf::ElementPtr modelElem;

  if (dPtr->modelSDF->Root()->HasElement("model"))
    modelElem = dPtr->modelSDF->Root()->GetElement("model");
  else if (dPtr->modelSDF->Root()->HasElement("light"))
    modelElem = dPtr->modelSDF->Root()->GetElement("light");
  else
  {
    gzerr << "No model or light in SDF\n";
    return false;
  }

  if (modelElem->HasElement("pose"))
    modelPose = modelElem->Get<ignition::math::Pose3d>("pose");

  modelName = dPtr->node->GetTopicNamespace() + "::" +
    modelElem->Get<std::string>("name");

  dPtr->modelVisual.reset(new rendering::Visual(modelName,
                          scene->GetWorldVisual()));
  dPtr->modelVisual->Load();
  dPtr->modelVisual->SetPose(modelPose);

  modelName = dPtr->modelVisual->GetName();
  modelElem->GetAttribute("name")->Set(modelName);

  if (modelElem->GetName() == "model")
  {
    sdf::ElementPtr linkElem = modelElem->GetElement("link");

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
              linkName, dPtr->modelVisual));
        linkVisual->Load();
        linkVisual->SetPose(linkPose);

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

          visualElem = visualElem->GetNextElement("visual");
        }

        linkElem = linkElem->GetNextElement("link");
      }
    }
    catch(common::Exception &_e)
    {
      return false;
    }
  }
  else if (modelElem->GetName() == "light")
  {
    dPtr->modelVisual->AttachMesh("unit_sphere");
  }
  else
  {
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void ModelMaker::Stop()
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  // Remove the temporary visual from the scene
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  scene->RemoveVisual(dPtr->modelVisual);
  dPtr->modelVisual.reset();
  dPtr->modelSDF.reset();

  EntityMaker::Stop();
}

/////////////////////////////////////////////////
void ModelMaker::CreateTheEntity()
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  msgs::Factory msg;
  if (!dPtr->clone)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    sdf::ElementPtr modelElem;
    bool isModel = false;
    bool isLight = false;
    if (dPtr->modelSDF->Root()->HasElement("model"))
    {
      modelElem = dPtr->modelSDF->Root()->GetElement("model");
      isModel = true;
    }
    else if (dPtr->modelSDF->Root()->HasElement("light"))
    {
      modelElem = dPtr->modelSDF->Root()->GetElement("light");
      isLight = true;
    }

    std::string modelName = modelElem->Get<std::string>("name");

    // Automatically create a new name if the model exists
    int i = 0;
    while ((isModel && has_entity_name(modelName)) ||
        (isLight && scene->GetLight(modelName)))
    {
      modelName = modelElem->Get<std::string>("name") + "_" +
        boost::lexical_cast<std::string>(i++);
    }

    // Remove the topic namespace from the model name. This will get re-inserted
    // by the World automatically
    modelName.erase(0, dPtr->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelName);
    modelElem->GetElement("pose")->Set(
        dPtr->modelVisual->GetWorldPose());

    // Spawn the model in the physics server
    msg.set_sdf(dPtr->modelSDF->ToString());
  }
  else
  {
    msgs::Set(msg.mutable_pose(), dPtr->modelVisual->GetWorldPose().Ign());
    msg.set_clone_model_name(dPtr->modelVisual->GetName().substr(0,
          dPtr->modelVisual->GetName().find("_clone_tmp")));
  }

  dPtr->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelMaker::EntityPosition() const
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  return dPtr->modelVisual->GetWorldPose().pos.Ign();
}

/////////////////////////////////////////////////
void ModelMaker::SetEntityPosition(const ignition::math::Vector3d &_pos)
{
  ModelMakerPrivate *dPtr =
      reinterpret_cast<ModelMakerPrivate *>(this->dataPtr);

  dPtr->modelVisual->SetWorldPosition(_pos);
}

