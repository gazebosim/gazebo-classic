/*
 * Copyright 2011 Nate Koenig
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
#include "gui/GuiEvents.hh"
#include "gui/ModelMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
  ModelMaker::ModelMaker()
: EntityMaker()
{
  this->state = 0;
  this->clone = false;
}

/////////////////////////////////////////////////
ModelMaker::~ModelMaker()
{
  this->camera.reset();
}

/////////////////////////////////////////////////
void ModelMaker::InitFromModel(const std::string &_modelName)
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  this->modelVisual = scene->CloneVisual(_modelName, _modelName + "_clone_tmp");

  if (!this->modelVisual)
    gzerr << "Unable to clone\n";

  this->clone = true;
}

/////////////////////////////////////////////////
void ModelMaker::InitFromSDFString(const std::string &_data)
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  this->modelSDF.reset(new sdf::SDF);
  sdf::initFile("gazebo.sdf", this->modelSDF);
  sdf::readString(_data, this->modelSDF);

  this->Init();
}


/////////////////////////////////////////////////
void ModelMaker::InitFromFile(const std::string &_filename)
{
  this->clone = false;
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  if (this->modelVisual)
  {
    scene->RemoveVisual(this->modelVisual);
    this->modelVisual.reset();
    this->visuals.clear();
  }

  this->modelSDF.reset(new sdf::SDF);
  sdf::initFile("gazebo.sdf", this->modelSDF);
  sdf::readFile(_filename, this->modelSDF);

  this->Init();
}

/////////////////////////////////////////////////
void ModelMaker::Init()
{
  rendering::Scene *scene = gui::get_active_camera()->GetScene();

  // Load the world file
  std::string modelName;
  math::Pose modelPose, linkPose, visualPose;
  sdf::ElementPtr modelElem;

  if (this->modelSDF->root->HasElement("model"))
    modelElem = this->modelSDF->root->GetElement("model");
  else if (this->modelSDF->root->HasElement("light"))
    modelElem = this->modelSDF->root->GetElement("light");

  if (modelElem->HasElement("pose"))
    modelPose = modelElem->GetValuePose("pose");

  modelName = this->node->GetTopicNamespace() + "::" +
    modelElem->GetValueString("name");

  this->modelVisual.reset(new rendering::Visual(modelName,
        scene->GetWorldVisual()));
  this->modelVisual->Load();
  this->modelVisual->SetPose(modelPose);

  modelName = this->modelVisual->GetName();
  modelElem->GetAttribute("name")->Set(modelName);

  scene->AddVisual(this->modelVisual);

  if (modelElem->GetName() == "model")
  {
    sdf::ElementPtr linkElem = modelElem->GetElement("link");

    try
    {
      while (linkElem)
      {
        std::string linkName = linkElem->GetValueString("name");
        if (linkElem->HasElement("pose"))
          linkPose = linkElem->GetValuePose("pose");
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
            visualPose = visualElem->GetValuePose("pose");
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
    }
  }
  else
  {
    this->modelVisual->AttachMesh("unit_sphere");
  }
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
  rendering::Scene *scene = gui::get_active_camera()->GetScene();
  scene->RemoveVisual(this->modelVisual);
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
  if (_event.button == common::MouseEvent::LEFT && !_event.dragging)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

/////////////////////////////////////////////////
void ModelMaker::OnMouseMove(const common::MouseEvent &_event)
{
  math::Pose pose = this->modelVisual->GetWorldPose();

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->camera->GetCameraToViewportRay(_event.pos.x, _event.pos.y,
      origin1, dir1);

  // Compute the distance from the camera to plane of translation
  math::Plane plane(math::Vector3(0, 0, 1), 0);

  double dist1 = plane.Distance(origin1, dir1);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  pose.pos = p1;

  if (!_event.shift)
  {
    if (ceil(pose.pos.x) - pose.pos.x <= .4)
      pose.pos.x = ceil(pose.pos.x);
    else if (pose.pos.x - floor(pose.pos.x) <= .4)
      pose.pos.x = floor(pose.pos.x);

    if (ceil(pose.pos.y) - pose.pos.y <= .4)
      pose.pos.y = ceil(pose.pos.y);
    else if (pose.pos.y - floor(pose.pos.y) <= .4)
      pose.pos.y = floor(pose.pos.y);

    /*  if (ceil(pose.pos.z) - pose.pos.z <= .4)
        pose.pos.z = ceil(pose.pos.z);
        else if (pose.pos.z - floor(pose.pos.z) <= .4)
        pose.pos.z = floor(pose.pos.z);
        */
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
    rendering::Scene *scene = gui::get_active_camera()->GetScene();
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

    std::string modelName = modelElem->GetValueString("name");

    // Automatically create a new name if the model exists
    int i = 0;
    while ((isModel && has_entity_name(modelName)) ||
        (isLight && scene->GetLight(modelName)))
    {
      modelName = modelElem->GetValueString("name") + "_" +
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
    msgs::Set(msg.mutable_pose(), this->modelVisual->GetWorldPose());
    msg.set_clone_model_name(this->modelVisual->GetName().substr(0,
          this->modelVisual->GetName().find("_clone_tmp")));
  }

  this->makerPub->Publish(msg);
}
