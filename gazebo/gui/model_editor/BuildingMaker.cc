/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <sstream>

#include "msgs/msgs.hh"

#include "common/Console.hh"
#include "common/MouseEvent.hh"
#include "common/Exception.hh"

#include "rendering/UserCamera.hh"
#include "rendering/Visual.hh"
#include "rendering/Scene.hh"

#include "math/Quaternion.hh"

#include "transport/Publisher.hh"
#include "transport/Node.hh"

#include "gui/Gui.hh"
#include "gui/EntityMaker.hh"
#include "gui/BoxMaker.hh"

#include "gui/model_editor/EditorEvents.hh"
#include "gui/model_editor/BuildingMaker.hh"


using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
  BuildingMaker::BuildingMaker() : EntityMaker()
{
  this->connections.push_back(
  gui::Events::ConnectCreateBuildingPart(
    boost::bind(&BuildingMaker::OnCreateBuildingPart, this, _1)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartPose(
    boost::bind(&BuildingMaker::OnSetBuildingPartPose, this, _1, _2)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartSize(
    boost::bind(&BuildingMaker::OnSetBuildingPartSize, this, _1, _2)));

  this->boxMaker = new BoxMaker;

  math::Pose modelPose;
  modelPose.Set(0, 0, 0, 0, 0, 0);
  this->MakeModel(modelPose);
}

/////////////////////////////////////////////////
BuildingMaker::~BuildingMaker()
{
//  this->camera.reset();
}

/////////////////////////////////////////////////
std::string BuildingMaker::MakeModel(math::Pose _pose)
{
  this->modelName = "";

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  math::Pose modelPose;

  modelPose = _pose;

  this->modelName = this->node->GetTopicNamespace() + "::" + "replace_me_with_name";

  this->modelVisual.reset(new rendering::Visual(modelName,
                          scene->GetWorldVisual()));
  this->modelVisual->Load();
  this->modelVisual->SetPose(modelPose);

  this->modelName = this->modelVisual->GetName();

  scene->AddVisual(this->modelVisual);

  return modelName;
}

/////////////////////////////////////////////////
void BuildingMaker::AddPart(std::string _type, math::Vector3 _size, math::Pose _pose)
{
  if (_type == "wall")
    this->AddWall(_size, _pose);
  else if (_type == "window")
    this->AddWindow(_size, _pose);
  else if (_type == "door")
    this->AddDoor(_size, _pose);
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWall(math::Vector3 _size, math::Pose _pose)
{
  std::string linkName = "Wall";

  math::Pose linkPose, visualPose;

  linkPose = _pose;

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();
  linkVisual->SetPose(linkPose);
  this->visuals.push_back(linkVisual);

  std::ostringstream visualName;
  visualName << modelName << "::" << linkName << "::Visual_"
    << this->wallVisuals.size();
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));

  std::string boxString = this->boxMaker->GetSDFString();

  sdf::ElementPtr visualElem;
  sdf::SDF sdf;
  sdf.SetFromString(boxString);
  if (sdf.root->HasElement("model"))
  {
    sdf::ElementPtr modelElem = sdf.root->GetElement("model");
    if (modelElem->HasElement("link"))
    {
      sdf::ElementPtr linkElem = modelElem->GetElement("link");
      if (linkElem->HasElement("visual"))
      {
        visualElem = linkElem->GetElement("visual");
        visVisual->Load(visualElem);
        visualPose.Set(0, 0, 0, 0, 0, 0);
        visVisual->SetPose(visualPose);
        this->visuals.push_back(visVisual);
        this->wallVisuals.push_back(visVisual);
      }
    }
  }

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(math::Vector3 _size, math::Pose _pose)
{
  std::string windowVisualName = "";
  return windowVisualName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(math::Vector3 _size, math::Pose _pose)
{
  std::string doorVisualName = "";
  return doorVisualName;
}

/////////////////////////////////////////////////
void BuildingMaker::SetPose(std::string _visualName, math::Pose _pose)
{
}

/////////////////////////////////////////////////
void BuildingMaker::SetSize(std::string _visualName, math::Vector3 _size)
{
}

/////////////////////////////////////////////////
void BuildingMaker::Start(const rendering::UserCameraPtr _camera)
{
}

/////////////////////////////////////////////////
void BuildingMaker::Stop()
{
}

/////////////////////////////////////////////////
bool BuildingMaker::IsActive() const
{
  return true;
}

/////////////////////////////////////////////////
void BuildingMaker::CreateTheEntity()
{
  /*msgs::Factory msg;
  if (!this->clone)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    sdf::ElementPtr modelElem;
    bool isModel = false;
    bool isLight = false;
    if (this->modelSDF->root->HasElement("model"))
    {
      modelElem = this->modelSDF->root->GetElement("model");
      isModel = true;
    }
    else if (this->modelSDF->root->HasElement("light"))
    {
      modelElem = this->modelSDF->root->GetElement("light");
      isLight = true;
    }

    std::string modelNameStr = modelElem->GetValueString("name");

    // Automatically create a new name if the model exists
    int i = 0;
    while ((isModel && has_entity_name(modelNameStr)) ||
        (isLight && scene->GetLight(modelNameStr)))
    {
      modelNameStr = modelElem->GetValueString("name") + "_" +
        boost::lexical_cast<std::string>(i++);
    }

    // Remove the topic namespace from the model name. This will get re-inserted
    // by the World automatically
    modelNameStr.erase(0, this->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelNameStr);
    modelElem->GetElement("pose")->Set(
        this->modelVisual->GetWorldPose());

    // Spawn the model in the physics server
    msg.set_sdf(this->modelSDF->ToString());
  }
  else
  {
    msgs::Set(msg.mutable_pose(), this->modelVisual->GetWorldPose());
    msg.set_clone_model_name(this->modelVisual->GetName().substr(0,
          this->modelVisual->GetName().find("_clone_tmp")));
  }

  this->makerPub->Publish(msg);*/
}

/////////////////////////////////////////////////
void BuildingMaker::OnCreateBuildingPart(std::string _partType)
{
  math::Vector3 size;
  math::Pose pose(0,0,0,0,0,0);
  this->AddPart(_partType, size, pose);
}

/////////////////////////////////////////////////
void BuildingMaker::OnSetBuildingPartPose(std::string _partName,
  math::Pose _pose)
{
}
/////////////////////////////////////////////////
/////////////////////////////////////////////////
void BuildingMaker::OnSetBuildingPartSize(std::string _partName,
  math::Vector3 _size)
{
}
