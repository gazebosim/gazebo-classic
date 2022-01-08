/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
/* Desc: Contact Visualization Class
 * Author: Nate Koenig
 */
#include <boost/bind/bind.hpp>

#include <ignition/common/Profiler.hh>

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/ContactVisualPrivate.hh"
#include "gazebo/rendering/ContactVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ContactVisual::ContactVisual(const std::string &_name, VisualPtr _vis,
                             const std::string &_topicName)
: Visual(*new ContactVisualPrivate, _name, _vis)
{
  ContactVisualPrivate *dPtr =
      reinterpret_cast<ContactVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_PHYSICS;

  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->Name());

  dPtr->topicName = _topicName;
  dPtr->contactsSub = dPtr->node->Subscribe(dPtr->topicName,
      &ContactVisual::OnContact, this);

  common::MeshManager::Instance()->CreateSphere("contact_sphere", 0.02, 10, 10);
  this->InsertMesh("contact_sphere");

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&ContactVisual::Update, this)));
}

/////////////////////////////////////////////////
ContactVisual::~ContactVisual()
{
}

/////////////////////////////////////////////////
void ContactVisual::Update()
{
  IGN_PROFILE("rendering::ContactVisual::Update");
  ContactVisualPrivate *dPtr =
      reinterpret_cast<ContactVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  if (!dPtr->contactsMsg || !dPtr->receivedMsg)
    return;

  // The following values are used to calculate normal scaling factor based
  // on force value.
  double magScale = 100;
  double vMax = 0.5;
  double vMin = 0.1;
  double vRange = vMax - vMin;
  double offset = vRange - vMin;

  unsigned int c = 0;
  for (int i = 0; i < dPtr->contactsMsg->contact_size(); i++)
  {
    for (int j = 0; j < dPtr->contactsMsg->contact(i).position_size(); j++)
    {
      auto pos = msgs::ConvertIgn(
          dPtr->contactsMsg->contact(i).position(j));
      auto normal = msgs::ConvertIgn(
          dPtr->contactsMsg->contact(i).normal(j));
      double depth = dPtr->contactsMsg->contact(i).depth(j);

      auto force = msgs::ConvertIgn(
          dPtr->contactsMsg->contact(i).wrench(j).body_1_wrench().force());

      // Scaling factor for the normal line.
      // Eq in the family of Y = 1/(1+exp(-(x^2)))
      double normalScale = (2.0 * vRange) / (1 + exp
          (-force.SquaredLength() / magScale)) - offset;

      // Create a new contact visualization point if necessary
      if (c >= dPtr->points.size())
        this->CreateNewPoint();

      dPtr->points[c]->contactPointVis->SetVisible(true);
      dPtr->points[c]->contactPointVis->SetPosition(pos);

      dPtr->points[c]->normal->SetPoint(1, (normal*normalScale));
      dPtr->points[c]->depth->SetPoint(1, (normal*-depth*10));

      GZ_OGRE_SET_MATERIAL_BY_NAME(dPtr->points[c]->normal, "Gazebo/LightOn");
      GZ_OGRE_SET_MATERIAL_BY_NAME(dPtr->points[c]->depth, "Gazebo/LightOff");
      dPtr->points[c]->depth->Update();
      dPtr->points[c]->normal->Update();
      c++;
    }
  }

  for ( ; c < dPtr->points.size(); c++)
    dPtr->points[c]->contactPointVis->SetVisible(false);

  dPtr->receivedMsg = false;
}

/////////////////////////////////////////////////
void ContactVisual::OnContact(ConstContactsPtr &_msg)
{
  ContactVisualPrivate *dPtr =
      reinterpret_cast<ContactVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);
  if (dPtr->enabled)
  {
    dPtr->contactsMsg = _msg;
    dPtr->receivedMsg = true;
  }
}

/////////////////////////////////////////////////
void ContactVisual::SetEnabled(bool _enabled)
{
  ContactVisualPrivate *dPtr =
      reinterpret_cast<ContactVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  dPtr->enabled = _enabled;

  if (!dPtr->enabled)
  {
    dPtr->contactsSub.reset();

    dPtr->contactsMsg.reset();
    dPtr->receivedMsg = false;

    for (unsigned int c = 0 ; c < dPtr->points.size(); c++)
      dPtr->points[c]->contactPointVis->SetVisible(false);
  }
  else if (!dPtr->contactsSub)
  {
    dPtr->contactsSub = dPtr->node->Subscribe(dPtr->topicName,
        &ContactVisual::OnContact, this);
  }
}

/////////////////////////////////////////////////
void ContactVisual::CreateNewPoint()
{
  ContactVisualPrivate *dPtr =
      reinterpret_cast<ContactVisualPrivate *>(this->dataPtr);

  std::string objName = this->Name() +
    "_contactpoint_" + std::to_string(dPtr->points.size());

  ContactVisualPrivate::ContactPoint *cp =
      new ContactVisualPrivate::ContactPoint();

  cp->contactPointVis.reset(
      new Visual(objName + "_node", shared_from_this(), false));
  cp->contactPointVis->Load();

  /// \todo We can improve this by using instanced geometry.
  cp->contactPointVis->AttachMesh("contact_sphere");
  cp->contactPointVis->SetMaterial("Gazebo/BlueLaser");

  cp->normal = cp->contactPointVis->CreateDynamicLine(RENDERING_LINE_LIST);
  cp->depth = cp->contactPointVis->CreateDynamicLine(RENDERING_LINE_LIST);

  cp->normal->AddPoint(ignition::math::Vector3d(0, 0, 0));
  cp->normal->AddPoint(ignition::math::Vector3d(0, 0, 0.1));

  cp->depth->AddPoint(ignition::math::Vector3d(0, 0, 0));
  cp->depth->AddPoint(ignition::math::Vector3d(0, 0, -1));

  cp->contactPointVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  cp->contactPointVis->SetVisible(false);

  dPtr->points.push_back(cp);
}
