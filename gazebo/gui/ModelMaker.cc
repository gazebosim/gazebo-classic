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
#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/math/Quaternion.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/ModelMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
  ModelMaker::ModelMaker()
: EntityMaker()
{
  this->state = 0;
  this->leftMousePressed = false;
  this->clone = false;
}

/////////////////////////////////////////////////
ModelMaker::~ModelMaker()
{
  this->camera.reset();
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromModel(const std::string & _modelName)
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  rendering::VisualPtr vis = scene->GetVisual(_modelName);
  if (!vis)
  {
    gzerr << "Model: '" << _modelName << "' does not exist." << std::endl;
    return false;
  }

  this->modelVisual = vis->Clone(
      _modelName + "_clone_tmp", scene->GetWorldVisual());

  if (!this->modelVisual)
  {
    gzerr << "Unable to clone\n";
    return false;
  }
  this->clone = true;
  return true;
}

/////////////////////////////////////////////////
bool ModelMaker::InitFromSDFString(const std::string &_data)
{
  this->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  this->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->modelSDF);
  sdf::readString(_data, this->modelSDF);

  if (!sdf::readString(_data, this->modelSDF))
  {
    gzerr << "Unable to load SDF from data\n";
    return false;
  }

  return this->Init();
}


/////////////////////////////////////////////////
bool ModelMaker::InitFromFile(const std::string &_filename)
{
  this->clone = false;
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  this->modelSDF.reset(new sdf::SDF);
  sdf::initFile("root.sdf", this->modelSDF);

  if (!sdf::readFile(_filename, this->modelSDF))
  {
    gzerr << "Unable to load file[" << _filename << "]\n";
    return false;
  }

  return this->Init();
}

/////////////////////////////////////////////////
bool ModelMaker::Init()
{
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  // Load the world file
  std::string modelName;
  math::Pose modelPose, linkPose, visualPose;
  sdf::ElementPtr modelElem;

  if (this->modelSDF->Root()->HasElement("model"))
    modelElem = this->modelSDF->Root()->GetElement("model");
  else if (this->modelSDF->Root()->HasElement("light"))
    modelElem = this->modelSDF->Root()->GetElement("light");
  else
  {
    gzerr << "No model or light in SDF\n";
    return false;
  }

  if (modelElem->HasElement("pose"))
    modelPose = modelElem->Get<math::Pose>("pose");

  modelName = this->node->GetTopicNamespace() + "::" +
    modelElem->Get<std::string>("name");

  this->modelVisual.reset(new rendering::Visual(modelName,
                          scene->GetWorldVisual()));
  this->modelVisual->Load();
  this->modelVisual->SetPose(modelPose);

  modelName = this->modelVisual->GetName();
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
          linkPose = linkElem->Get<math::Pose>("pose");
        else
          linkPose.Set(0, 0, 0, 0, 0, 0);

        rendering::VisualPtr linkVisual(new rendering::Visual(modelName + "::" +
              linkName, this->modelVisual));
        linkVisual->Load();
        linkVisual->SetPose(linkPose);
        this->visuals.push_back(linkVisual);

        int visualIndex = 0;
        sdf::ElementPtr visualElem;

        if (linkElem->HasElement("visual"))
          visualElem = linkElem->GetElement("visual");

        while (visualElem)
        {
          if (visualElem->HasElement("pose"))
            visualPose = visualElem->Get<math::Pose>("pose");
          else
            visualPose.Set(0, 0, 0, 0, 0, 0);

          std::ostringstream visualName;
          visualName << modelName << "::" << linkName << "::Visual_"
            << visualIndex++;
          rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
                linkVisual));

          visVisual->Load(visualElem);
          visVisual->SetPose(visualPose);
          this->visuals.push_back(visVisual);

          visualElem = visualElem->GetNextElement("visual");
        }

        linkElem = linkElem->GetNextElement("link");
      }
    }
    catch(common::Exception &_e)
    {
      this->visuals.clear();
      return false;
    }
  }
  else if (modelElem->GetName() == "light")
  {
    this->modelVisual->AttachMesh("unit_sphere");
  }
  else
  {
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void ModelMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;
  this->state = 1;
}

/////////////////////////////////////////////////
void ModelMaker::Stop()
{
  // Remove the temporary visual from the scene
  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
  for (auto vis : this->visuals)
    scene->RemoveVisual(vis);
  this->modelVisual.reset();
  this->visuals.clear();
  this->modelSDF.reset();

  this->state = 0;
  gui::Events::moveMode(true);
}

/////////////////////////////////////////////////
bool ModelMaker::IsActive() const
{
  return this->state > 0;
}

/////////////////////////////////////////////////
void ModelMaker::OnMousePush(const common::MouseEvent &/*_event*/)
{
}

/////////////////////////////////////////////////
void ModelMaker::OnMouseRelease(const common::MouseEvent &_event)
{
  if (_event.Button() == common::MouseEvent::LEFT)
  {
    // Place if not dragging, or if dragged for less than 50 pixels.
    // The 50 pixels is used to account for accidental mouse movement
    // when placing an object.
    if (!_event.Dragging() || _event.PressPos().Distance(_event.Pos()) < 50)
    {
      this->CreateTheEntity();
      this->Stop();
    }
  }
}

/////////////////////////////////////////////////
void ModelMaker::OnMouseMove(const common::MouseEvent &_event)
{
  math::Pose pose = this->modelVisual->GetWorldPose();
  pose.pos = ModelManipulator::GetMousePositionOnPlane(this->camera, _event);

  if (!_event.Shift())
  {
    pose.pos = ModelManipulator::SnapPoint(pose.pos);
  }
  pose.pos.z = this->modelVisual->GetWorldPose().pos.z;

  this->modelVisual->SetWorldPose(pose);
}

/////////////////////////////////////////////////
void ModelMaker::OnMouseDrag(const common::MouseEvent &/*_event*/)
{
}

/////////////////////////////////////////////////
void ModelMaker::CreateTheEntity()
{
  msgs::Factory msg;
  if (!this->clone)
  {
    rendering::ScenePtr scene = gui::get_active_camera()->GetScene();
    sdf::ElementPtr modelElem;
    bool isModel = false;
    bool isLight = false;
    if (this->modelSDF->Root()->HasElement("model"))
    {
      modelElem = this->modelSDF->Root()->GetElement("model");
      isModel = true;
    }
    else if (this->modelSDF->Root()->HasElement("light"))
    {
      modelElem = this->modelSDF->Root()->GetElement("light");
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
    modelName.erase(0, this->node->GetTopicNamespace().size()+2);

    // The the SDF model's name
    modelElem->GetAttribute("name")->Set(modelName);
    modelElem->GetElement("pose")->Set(
        this->modelVisual->GetWorldPose());

    // Spawn the model in the physics server
    msg.set_sdf(this->modelSDF->ToString());
  }
  else
  {
    msgs::Set(msg.mutable_pose(), this->modelVisual->GetWorldPose().Ign());
    msg.set_clone_model_name(this->modelVisual->GetName().substr(0,
          this->modelVisual->GetName().find("_clone_tmp")));
  }

  this->makerPub->Publish(msg);
}
