/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/MeshManager.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RayQuery.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/ModelSnapPrivate.hh"
#include "gazebo/gui/ModelSnap.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelSnap::ModelSnap()
  : dataPtr(new ModelSnapPrivate)
{
  this->dataPtr->initialized = false;

  this->dataPtr->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
ModelSnap::~ModelSnap()
{
  this->dataPtr->modelPub.reset();

  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->snapVisual->DeleteDynamicLine(this->dataPtr->snapLines);
    this->dataPtr->snapVisual.reset();
  }

  event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();

  delete this->dataPtr->updateMutex;

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ModelSnap::Init()
{
  if (this->dataPtr->initialized)
    return;

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene =  cam->GetScene();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->modelPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->rayQuery.reset(
      new rendering::RayQuery(this->dataPtr->userCamera));

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelSnap::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedTriangle.clear();

  if (this->dataPtr->snapVisual)
  {
    this->dataPtr->snapVisual->SetVisible(false);
    if (this->dataPtr->snapVisual->GetParent())
    {
      this->dataPtr->snapVisual->GetParent()->DetachVisual(
          this->dataPtr->snapVisual);
    }
  }

  event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();
}

/////////////////////////////////////////////////
void ModelSnap::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void ModelSnap::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.pos);
  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
    if (!this->dataPtr->selectedVis)
    {
      math::Vector3 intersect;
      std::vector<math::Vector3> worldTriangle;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis->GetRootVisual(), intersect, worldTriangle);


      if (!worldTriangle.empty())
      {
        this->dataPtr->selectedVis = vis;

        this->dataPtr->selectedTriangle.clear();
        for (unsigned int i = 0; i < worldTriangle.size(); ++i)
        {
          this->dataPtr->selectedTriangle.push_back(
              this->dataPtr->selectedVis->GetWorldPose().rot.GetInverse() *
              (worldTriangle[i] -
              this->dataPtr->selectedVis->GetWorldPose().pos));
        }

        /*std::cerr << " got first tri " << intersect << std::endl;
        std::cerr << this->dataPtr->selectedTriangle[0] << std::endl;
        std::cerr << this->dataPtr->selectedTriangle[1] << std::endl;
        std::cerr << this->dataPtr->selectedTriangle[2] << std::endl;
        std::cerr << "first normal " << math::Vector3::GetNormal(
            this->dataPtr->selectedTriangle[0],
            this->dataPtr->selectedTriangle[1],
            this->dataPtr->selectedTriangle[2]) << std::endl;*/

        this->dataPtr->renderConnection = event::Events::ConnectRender(
              boost::bind(&ModelSnap::Update, this));
      }
    }
    else if (vis != this->dataPtr->selectedVis)
    {
      math::Vector3 intersect;
      std::vector<math::Vector3> vertices;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis->GetRootVisual(), intersect, vertices);

      if (!vertices.empty())
      {
        // std::cerr << " got second tri " << intersect << std::endl;
        // std::cerr << vertices[0] << std::endl;
        // std::cerr << vertices[1] << std::endl;
        // std::cerr << vertices[2] << std::endl;
        //std::cerr << "2nd normal " << math::Vector3::GetNormal(
        //    vertices[0], vertices[1], vertices[2]) << std::endl;

        std::vector<math::Vector3> worldTriangle;
        {
          boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
          for (unsigned int i = 0; i < this->dataPtr->selectedTriangle.size();
              ++i)
          {
            worldTriangle.push_back(
                this->dataPtr->selectedVis->GetWorldPose().pos +
                this->dataPtr->selectedVis->GetWorldPose().rot *
                this->dataPtr->selectedTriangle[i]);
          }
        }

        math::Vector3 centroidA = (worldTriangle[0] + worldTriangle[1] +
            worldTriangle[2]) / 3.0;

        math::Vector3 centroidB =
            (vertices[0] + vertices[1] + vertices[2]) / 3.0;

//        std::cerr << "centroidA " << centroidA << std::endl;
//        std::cerr << "centroidB " << centroidB << std::endl;
        math::Vector3 normalA = math::Vector3::GetNormal(
            worldTriangle[0], worldTriangle[1], worldTriangle[2]);

        math::Vector3 normalB = math::Vector3::GetNormal(
            vertices[0], vertices[1], vertices[2]);

        math::Vector3 u = normalB.Normalize() * -1;
        math::Vector3 v = normalA.Normalize();
        double cosTheta = v.Dot(u);
        double angle = acos(cosTheta);
        math::Quaternion rotation;
        // check the parallel case
        if (math::equal(angle, M_PI))
          rotation.SetFromAxis(u.GetPerpendicular(), angle);
        else
          rotation.SetFromAxis((v.Cross(u)).Normalize(), angle);

        rendering::VisualPtr modelVis =
            this->dataPtr->selectedVis->GetRootVisual();

        // Get translation needed for alignment
        // taking into account the rotated position of the mesh
        math::Vector3 translation = centroidB -
            (rotation * (centroidA - modelVis->GetWorldPose().pos) +
            modelVis->GetWorldPose().pos);


        /*std::cerr << "angle " << angle << std::endl;
        std::cerr << "q " << rotation.GetAsEuler() << std::endl;
        std::cerr << "translation " << translation << std::endl;
        std::cerr << "centroidA " << centroidA << std::endl;
        std::cerr << "modelVis->GetWorldPose().pos " << modelVis->GetWorldPose().pos << std::endl;*/

        modelVis->SetWorldPose(
            math::Pose(modelVis->GetWorldPose().pos + translation,
            rotation * modelVis->GetWorldPose().rot));

        this->PublishVisualPose(modelVis);

        this->Reset();
        gui::Events::manipMode("select");
      }
    }
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::OnKeyPressEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;
}

//////////////////////////////////////////////////
void ModelSnap::OnKeyReleaseEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;

  this->dataPtr->keyEvent.key = 0;
}

/////////////////////////////////////////////////
void ModelSnap::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // Check to see if the visual is a model.
  if (gui::get_entity_id(_vis->GetName()))
  {
    msgs::Model msg;
    msg.set_id(gui::get_entity_id(_vis->GetName()));
    msg.set_name(_vis->GetName());

    msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
    this->dataPtr->modelPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void ModelSnap::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  if (this->dataPtr->selectedTriangle.empty())
    return;

  if (!this->dataPtr->snapVisual)
  {
    rendering::UserCameraPtr camera = gui::get_active_camera();

    std::string snapVisName = "_SNAP_";
    this->dataPtr->snapVisual.reset(new rendering::Visual(
        snapVisName, this->dataPtr->selectedVis, false));

    this->dataPtr->snapLines =
        this->dataPtr->snapVisual->CreateDynamicLine(
        rendering::RENDERING_LINE_STRIP);

    this->dataPtr->snapLines->setMaterial("Gazebo/RedGlow");
    this->dataPtr->snapLines->AddPoint(this->dataPtr->selectedTriangle[0]);
    this->dataPtr->snapLines->AddPoint(this->dataPtr->selectedTriangle[1]);
    this->dataPtr->snapLines->AddPoint(this->dataPtr->selectedTriangle[2]);
    this->dataPtr->snapLines->AddPoint(this->dataPtr->selectedTriangle[0]);
    this->dataPtr->snapVisual->SetVisible(true);
    this->dataPtr->snapVisual->GetSceneNode()->setInheritScale(false);
    return;
  }

  if (!this->dataPtr->snapVisual->GetVisible())
  {
    this->dataPtr->selectedVis->AttachVisual(this->dataPtr->snapVisual);
    this->dataPtr->snapLines->SetPoint(0, this->dataPtr->selectedTriangle[0]);
    this->dataPtr->snapLines->SetPoint(1, this->dataPtr->selectedTriangle[1]);
    this->dataPtr->snapLines->SetPoint(2, this->dataPtr->selectedTriangle[2]);
    this->dataPtr->snapLines->SetPoint(3, this->dataPtr->selectedTriangle[0]);

    this->dataPtr->snapVisual->SetVisible(true);
  }

  return;
}
