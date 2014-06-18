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

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
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
}

/////////////////////////////////////////////////
ModelSnap::~ModelSnap()
{
  this->dataPtr->modelPub.reset();
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
  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedTriangle.clear();
}

/////////////////////////////////////////////////
void ModelSnap::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  //this->dataPtr->mouseStart = _event.pressPos;
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  rendering::VisualPtr mouseVis
      = this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.pos);

  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
  }
  else
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
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis, intersect, this->dataPtr->selectedTriangle);
      if (!this->dataPtr->selectedTriangle.empty())
      {
        this->dataPtr->selectedVis = vis;
        // std::cerr << " got first tri " << intersect << std::endl;
        // std::cerr << this->dataPtr->selectedTriangle[0] << std::endl;
        // std::cerr << this->dataPtr->selectedTriangle[1] << std::endl;
        // std::cerr << this->dataPtr->selectedTriangle[2] << std::endl;
      }
    }
    else if (vis != this->dataPtr->selectedVis)
    {
      math::Vector3 intersect;
      std::vector<math::Vector3> vertices;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis, intersect, vertices);

      if (!vertices.empty())
      {
        // std::cerr << " got second tri " << intersect << std::endl;
        // std::cerr << vertices[0] << std::endl;
        // std::cerr << vertices[1] << std::endl;
        // std::cerr << vertices[2] << std::endl;

        math::Vector3 centroidA = (this->dataPtr->selectedTriangle[0] +
            this->dataPtr->selectedTriangle[1] +
            this->dataPtr->selectedTriangle[2]) / 3.0;

        math::Vector3 centroidB =
            (vertices[0] + vertices[1] + vertices[2]) / 3.0;

//        std::cerr << "centroidA " << centroidA << std::endl;
//        std::cerr << "centroidB " << centroidB << std::endl;

        math::Vector3 normalA = math::Vector3::GetNormal(
            this->dataPtr->selectedTriangle[0],
            this->dataPtr->selectedTriangle[1],
            this->dataPtr->selectedTriangle[2]);
        math::Vector3 normalB = math::Vector3::GetNormal(
            vertices[0], vertices[1], vertices[2]);

        math::Vector3 u = normalB.Normalize() * -1;
        math::Vector3 v = normalA.Normalize();
        double cosTheta = v.Dot(u);
        double angle = acos(cosTheta);
        math::Vector3 w = (v.Cross(u)).Normalize();
        math::Quaternion rotation;
        rotation.SetFromAxis(w, angle);

        rendering::VisualPtr modelVis =
            this->dataPtr->selectedVis->GetRootVisual();

        // Get translation of alignment, taking into account the rotated
        // position of the mesh
        math::Vector3 translation = centroidB -
            (rotation * (centroidA - modelVis->GetWorldPose().pos) +
            modelVis->GetWorldPose().pos);
//        std::cerr << this->dataPtr->selectedVis->GetName() << " " << translation  << std::endl;
//        std::cerr << "centroid A " <<  centroidA << std::endl;
//        std::cerr << "Rotated centroid A  " <<
//            (rotation * (centroidA - modelVis->GetWorldPose().pos) +
//            modelVis->GetWorldPose().pos)  << std::endl;

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
void ModelSnap::SetMouseMoveVisual(rendering::VisualPtr _vis)
{
  this->dataPtr->mouseMoveVis = _vis;
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
