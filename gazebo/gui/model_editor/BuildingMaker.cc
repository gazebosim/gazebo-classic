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

#include "gui/model_editor/EditorItem.hh"


using namespace gazebo;
using namespace gui;

double BuildingMaker::conversionScale;

/////////////////////////////////////////////////
WallManip::WallManip()
{
}

/////////////////////////////////////////////////
WallManip::~WallManip()
{
}

/////////////////////////////////////////////////
void WallManip::SetVisual(rendering::VisualPtr _visual)
{
  this->visual = _visual;
}

/////////////////////////////////////////////////
void WallManip::OnSizeChanged(double _width, double _length, double _height)
{
  this->size = BuildingMaker::ConvertSize(_length, _width, _height);
  math::Vector3 dScale = this->visual->GetScale() - this->size;
  this->visual->SetScale(this->size);

  // adjust position due to difference in pivot points
  math::Vector3 newPos = this->visual->GetPosition()
      - math::Vector3(dScale.x/2.0 + dScale.y/2.0, 0, dScale.z/2.0);
  this->visual->SetPosition(newPos);
}

/////////////////////////////////////////////////
void WallManip::OnPoseChanged(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  math::Pose newPose = BuildingMaker::ConvertPose(_x, _y, _z, _roll, _pitch,
      _yaw);
  this->visual->GetParent()->SetWorldPose(newPose);
}

/////////////////////////////////////////////////
  BuildingMaker::BuildingMaker() : EntityMaker()
{
/*  this->connections.push_back(
  gui::Events::ConnectCreateBuildingPart(
    boost::bind(&BuildingMaker::OnCreateBuildingPart, this, _1)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartPose(
    boost::bind(&BuildingMaker::OnSetBuildingPartPose, this, _1, _2)));

  this->connections.push_back(
  gui::Events::ConnectSetBuildingPartSize(
    boost::bind(&BuildingMaker::OnSetBuildingPartSize, this, _1, _2)));*/

  this->boxMaker = new BoxMaker;

  this->conversionScale = 0.01;

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
void BuildingMaker::ConnectItem(std::string _partName, EditorItem *_item)
{
  WallManip *manip = this->walls[_partName];
  QObject::connect(_item, SIGNAL(sizeChanged(double, double, double)),
      manip, SLOT(OnSizeChanged(double, double, double)));
  QObject::connect(_item, SIGNAL(poseChanged(double, double, double,
      double, double, double)), manip, SLOT(OnPoseChanged(double, double,
      double, double, double, double)));
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
void BuildingMaker::AddPart(std::string _type, math::Vector3 _size,
    math::Pose _pose)
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
    << this->walls.size();
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
        visVisual->GetParent()->SetPose(_pose);
        /*qDebug() << "size " << _size.x << _size.y << _size.z;
        qDebug() << "pos " << _pose.pos.x << _pose.pos.y << _pose.pos.z;
        math::Vector3 rotate = _pose.rot.GetAsEuler();
        qDebug() << "rot " << rotate.x << rotate.y << rotate.z;*/

        visVisual->SetScale(_size);
        this->visuals.push_back(visVisual);
        WallManip *wallManip = new WallManip();
        wallManip->SetVisual(visVisual);
        this->wallManips.push_back(wallManip);
        this->walls[visualName.str()] = wallManip;
      }
    }
  }

  return visualName.str();
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddWindow(math::Vector3 /*_size*/, math::Pose /*_pose*/)
{
  std::string windowVisualName = "";
  return windowVisualName;
}

/////////////////////////////////////////////////
std::string BuildingMaker::AddDoor(math::Vector3 /*_size*/, math::Pose /*_pose*/)
{
  std::string doorVisualName = "";
  return doorVisualName;
}

/////////////////////////////////////////////////
void BuildingMaker::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;
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
math::Vector3 BuildingMaker::ConvertSize(double _width, double _length,
    double _height)
{
  return math::Vector3(conversionScale*_width, conversionScale*_length,
      conversionScale*_height);
}

/////////////////////////////////////////////////
math::Pose BuildingMaker::ConvertPose(double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw)
{
  return math::Pose(conversionScale*_x, -conversionScale*_y, conversionScale*_z,
      GZ_DTOR(_roll), GZ_DTOR(_pitch), GZ_DTOR(_yaw));
}

/////////////////////////////////////////////////
math::Vector3 BuildingMaker::ConvertSize(QVector3D _size)
{
  QVector3D scaledSize = conversionScale*_size;
  return math::Vector3(scaledSize.x(), scaledSize.y(), scaledSize.z());
}

/////////////////////////////////////////////////
math::Pose BuildingMaker::ConvertPose(QVector3D _pos, QVector3D _rot)
{
  QVector3D scaledPos = conversionScale*_pos;
  return math::Pose(scaledPos.x(), -scaledPos.y(), scaledPos.z(),
      GZ_DTOR(_rot.x()), GZ_DTOR(_rot.y()), GZ_DTOR(_rot.z()));
}
