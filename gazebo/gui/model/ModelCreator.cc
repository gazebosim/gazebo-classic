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
#include <boost/filesystem.hpp>

#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/math/Quaternion.hh"

#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/EntityMaker.hh"

#include "gazebo/gui/model/ModelCreator.hh"


using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelCreator::ModelCreator() : EntityMaker()
{
  this->modelName = "";

  this->modelTemplateSDF.reset(new sdf::SDF);
  this->modelTemplateSDF->SetFromString(this->GetTemplateSDFString());

/*  this->connections.push_back(
  gui::editor::Events::ConnectSaveBuildingEditor(
    boost::bind(&ModelCreator::OnSave, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectDiscardBuildingEditor(
    boost::bind(&ModelCreator::OnDiscard, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectDoneBuildingEditor(
    boost::bind(&ModelCreator::OnDone, this)));
  this->connections.push_back(
  gui::editor::Events::ConnectExitBuildingEditor(
    boost::bind(&ModelCreator::OnExit, this)));

  this->saveDialog =
      new FinishBuildingDialog(FinishBuildingDialog::MODEL_SAVE, 0);
  this->finishDialog =
      new FinishBuildingDialog(FinishBuildingDialog::MODEL_FINISH, 0);*/

  this->boxCounter = 0;
  this->cylinderCounter = 0;
  this->sphereCounter = 0;

  this->Reset();
}

/////////////////////////////////////////////////
ModelCreator::~ModelCreator()
{
  this->camera.reset();
/*  if (this->saveDialog)
    delete this->saveDialog;
  if (this->finishDialog)
    delete this->finishDialog;*/
}

/////////////////////////////////////////////////
std::string ModelCreator::CreateModel()
{
  this->Reset();
  return this->modelName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddBox(const math::Vector3 &_size,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "unit_box_" << this->boxCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/OrangeTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("box"))->AddElement("size"))->Set(_size);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _size.z/2));
  }

  this->allParts[visVisual->GetName()] = visVisual;
  this->mouseVisual = linkVisual;
  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddSphere(double _radius,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "unit_sphere_" << this->sphereCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/OrangeTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  ((geomElem->AddElement("sphere"))->GetElement("radius"))->Set(_radius);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _radius));
  }

  this->allParts[visVisual->GetName()] = visVisual;
  this->mouseVisual = linkVisual;

  return linkName;
}

/////////////////////////////////////////////////
std::string ModelCreator::AddCylinder(double _radius, double _length,
    const math::Pose &_pose)
{
  if (!this->modelVisual)
    this->Reset();

  std::ostringstream linkNameStream;
  linkNameStream << "unit_cylinder_" << this->cylinderCounter++;
  std::string linkName = linkNameStream.str();

  rendering::VisualPtr linkVisual(new rendering::Visual(this->modelName + "::" +
        linkName, this->modelVisual));
  linkVisual->Load();

  std::ostringstream visualName;
  visualName << this->modelName << "::" << linkName << "::Visual";
  rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        linkVisual));
  sdf::ElementPtr visualElem =  this->modelTemplateSDF->root
      ->GetElement("model")->GetElement("link")->GetElement("visual");
  visualElem->GetElement("material")->GetElement("script")
      ->GetElement("name")->Set("Gazebo/OrangeTransparent");

  sdf::ElementPtr geomElem =  visualElem->GetElement("geometry");
  geomElem->ClearElements();
  sdf::ElementPtr cylinderElem = geomElem->AddElement("cylinder");
  (cylinderElem->GetElement("radius"))->Set(_radius);
  (cylinderElem->GetElement("length"))->Set(_length);

  visVisual->Load(visualElem);

  linkVisual->SetPose(_pose);
  if (_pose == math::Pose::Zero)
  {
    linkVisual->SetPosition(math::Vector3(_pose.pos.x, _pose.pos.y,
    _pose.pos.z + _length/2));
  }

  this->allParts[visVisual->GetName()] = visVisual;
  this->mouseVisual = linkVisual;

  return linkName;
}


/////////////////////////////////////////////////
void ModelCreator::RemovePart(const std::string &_partName)
{
  if (!this->modelVisual)
    this->Reset();

  rendering::VisualPtr vis = this->allParts[_partName];
  if (!vis)
  {
    gzerr << _partName << " does not exist\n";
    return;
  }
  rendering::VisualPtr visParent = vis->GetParent();
  rendering::ScenePtr scene = vis->GetScene();
  scene->RemoveVisual(vis);
  if (visParent)
    scene->RemoveVisual(visParent);
  this->allParts.erase(_partName);
}

/////////////////////////////////////////////////
void ModelCreator::Start(const rendering::UserCameraPtr _camera)
{
  this->camera = _camera;
}

/////////////////////////////////////////////////
void ModelCreator::Stop()
{
}

/////////////////////////////////////////////////
void ModelCreator::Reset()
{
  this->saved = false;
  this->saveLocation = QDir::homePath().toStdString();
  this->modelName = "default_model";

  if (!gui::get_active_camera() ||
      !gui::get_active_camera()->GetScene())
    return;

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  if (this->modelVisual)
    scene->RemoveVisual(this->modelVisual);

  this->modelVisual.reset(new rendering::Visual(this->modelName,
      scene->GetWorldVisual()));

  this->modelVisual->Load();
  this->modelPose = math::Pose::Zero;
  this->modelVisual->SetPose(this->modelPose);
  scene->AddVisual(this->modelVisual);

  while (this->allParts.size() > 0)
  {
    this->RemovePart(this->allParts.begin()->first);
  }
  this->allParts.clear();

  MouseEventHandler::Instance()->AddPressFilter("model_part",
      boost::bind(&ModelCreator::OnMousePressPart, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("model_part",
      boost::bind(&ModelCreator::OnMouseMovePart, this, _1));

  MouseEventHandler::Instance()->AddDoubleClickFilter("model_part",
      boost::bind(&ModelCreator::OnMouseDoubleClickPart, this, _1));
}

/////////////////////////////////////////////////
bool ModelCreator::IsActive() const
{
  return true;
}

/////////////////////////////////////////////////
void ModelCreator::SetModelName(const std::string &_modelName)
{
  this->modelName = _modelName;
}

/////////////////////////////////////////////////
void ModelCreator::SaveToSDF(const std::string &_savePath)
{
  this->saveLocation = _savePath;
  std::ofstream savefile;
  boost::filesystem::path path;
  path = boost::filesystem::operator/(this->saveLocation,
      this->modelName + ".sdf");
  savefile.open(path.string().c_str());
  savefile << this->modelSDF->ToString();
  savefile.close();
}

/////////////////////////////////////////////////
void ModelCreator::FinishModel()
{
  this->CreateTheEntity();
  this->Stop();
}

/////////////////////////////////////////////////
void ModelCreator::GenerateSDF()
{

}

/////////////////////////////////////////////////
void ModelCreator::CreateTheEntity()
{
  this->GenerateSDF();
  msgs::Factory msg;
  msg.set_sdf(this->modelSDF->ToString());
  this->makerPub->Publish(msg);
}

/////////////////////////////////////////////////
std::string ModelCreator::GetTemplateSDFString()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='template_model'>"
    << "<pose>0 0 0.0 0 0 0</pose>"
    << "<link name ='link'>"
    <<   "<visual name ='visual'>"
    <<     "<pose>0 0 0.0 0 0 0</pose>"
    <<     "<geometry>"
    <<       "<box>"
    <<         "<size>1.0 1.0 1.0</size>"
    <<       "</box>"
    <<     "</geometry>"
    <<     "<material>"
    <<       "<script>"
    <<         "<uri>file://media/materials/scripts/gazebo.material</uri>"
    <<         "<name>Gazebo/Grey</name>"
    <<       "</script>"
    <<     "</material>"
    <<   "</visual>"
    << "</link>"
    << "<static>true</static>"
    << "</model>"
    << "</sdf>";

  return newModelStr.str();
}


/////////////////////////////////////////////////
void ModelCreator::CreatePart(PartType _type)
{
  this->createPartType = _type;
  if (_type != PART_NONE)
  {
    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
/*    MouseEventHandler::Instance()->AddPressFilter("model_part",
        boost::bind(&ModelCreator::OnMousePressPart, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("model_part",
        boost::bind(&ModelCreator::OnMouseMovePart, this, _1));*/

    switch (_type)
    {
      case PART_BOX:
      {
        this->AddBox();
        break;
      }
      case PART_SPHERE:
      {
        this->AddSphere();
        break;
      }
      case PART_CYLINDER:
      {
        this->AddCylinder();
        break;
      }
      default:
        break;
    }
  }
  else
  {
    // Remove the event filters.
//    MouseEventHandler::Instance()->RemovePressFilter("model_part");
//    MouseEventHandler::Instance()->RemoveMoveFilter("model_part");
  }
}


/////////////////////////////////////////////////
bool ModelCreator::OnMousePressPart(const common::MouseEvent &_event)
{
  if (!this->mouseVisual || _event.button != common::MouseEvent::LEFT)
    return false;
  else
  {
    this->mouseVisual.reset();
    this->CreatePart(PART_NONE);
    return true;
  }
}

/////////////////////////////////////////////////
bool ModelCreator::OnMouseMovePart(const common::MouseEvent &_event)
{
  if (!this->mouseVisual)
    return false;

  if (!gui::get_active_camera())
    return false;

  math::Pose pose = this->mouseVisual->GetWorldPose();

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;


  // Cast two rays from the camera into the world
  gui::get_active_camera()->GetCameraToViewportRay(_event.pos.x, _event.pos.y,
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

  }
  pose.pos.z = this->mouseVisual->GetWorldPose().pos.z;

  this->mouseVisual->SetWorldPose(pose);

  return true;
}


/////////////////////////////////////////////////
bool ModelCreator::OnMouseDoubleClickPart(const common::MouseEvent &_event)
{
  rendering::VisualPtr vis = gui::get_active_camera()->GetVisual(_event.pos);
  if (vis)
  {
    if (this->allParts.find(vis->GetName()) !=
        this->allParts.end())
    {
      gzerr << " got model part " << vis->GetName() <<  std::endl;
      return true;
    }
  }
  return false;
}

/*
/////////////////////////////////////////////////
void ModelCreator::OnDiscard()
{
  int ret = QMessageBox::warning(0, QString("Discard"),
      QString("Are you sure you want to discard\n"
      "your model? All of your work will\n"
      "be lost."),
      QMessageBox::Discard | QMessageBox::Cancel,
      QMessageBox::Cancel);

  switch (ret)
  {
    case QMessageBox::Discard:
      gui::editor::Events::discardBuildingModel();
      this->modelName = this->buildingDefaultName;
      this->saveLocation = QDir::homePath().toStdString();
      this->saved = false;
      break;
    case QMessageBox::Cancel:
    // Do nothing
    break;
    default:
    break;
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnSave()
{
  if (this->saved)
  {
    this->SetModelName(this->modelName);
    this->GenerateSDF();
    this->SaveToSDF(this->saveLocation);
  }
  else
  {
    this->saveDialog->SetModelName(this->modelName);
    this->saveDialog->SetSaveLocation(this->saveLocation);
    if (this->saveDialog->exec() == QDialog::Accepted)
    {
      this->modelName = this->saveDialog->GetModelName();
      this->saveLocation = this->saveDialog->GetSaveLocation();
      this->SetModelName(this->modelName);
      this->GenerateSDF();
      this->SaveToSDF(this->saveLocation);
      this->saved = true;
    }
  }
  gui::editor::Events::saveBuildingModel(this->modelName, this->saveLocation);
}

/////////////////////////////////////////////////
void ModelCreator::OnDone()
{
  this->finishDialog->SetModelName(this->modelName);
  this->finishDialog->SetSaveLocation(this->saveLocation);
  if (this->finishDialog->exec() == QDialog::Accepted)
  {
    this->modelName = this->finishDialog->GetModelName();
    this->saveLocation = this->finishDialog->GetSaveLocation();
    this->SetModelName(this->modelName);
    this->GenerateSDF();
    this->SaveToSDF(this->saveLocation);
    this->FinishModel();
    gui::editor::Events::discardBuildingModel();
    gui::editor::Events::finishBuildingModel();
  }
}

/////////////////////////////////////////////////
void ModelCreator::OnExit()
{
  int ret = QMessageBox::warning(0, QString("Exit"),
      QString("Save Changes before exiting? If you do not\n"
      "save, all of your work will be lost!\n\n"
      "Note: Building Editor state is not maintained\n"
      "between Gazebo sessions. Once you quit\n"
      "Gazebo, your building will no longer be editable.\n\n"),
//      "If you are done editing your model, select Done\n"),
      QMessageBox::Discard | QMessageBox::Cancel | QMessageBox::Save,
      QMessageBox::Save);

  switch (ret)
  {
    case QMessageBox::Discard:
      gui::editor::Events::discardBuildingModel();
      this->modelName = this->buildingDefaultName;
      this->saveLocation = QDir::homePath().toStdString();
      this->saved = false;
      gui::editor::Events::finishBuildingModel();
      break;
    case QMessageBox::Cancel:
      break;
    case QMessageBox::Save:
      this->OnSave();
      gui::editor::Events::finishBuildingModel();
      break;
    default:
      break;
  }
}*/
