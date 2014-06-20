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
/* Desc: Contact Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/ContactVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ContactVisual::ContactVisual(const std::string &_name, VisualPtr _vis,
                             const std::string &_topicName)
: Visual(_name, _vis)
{
  this->receivedMsg = false;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->topicName = _topicName;
  this->contactsSub = this->node->Subscribe(this->topicName,
      &ContactVisual::OnContact, this);

  common::MeshManager::Instance()->CreateSphere("contact_sphere", 0.02, 10, 10);

  // Add the mesh into OGRE
  if (!this->sceneNode->getCreator()->hasEntity("contact_sphere") &&
      common::MeshManager::Instance()->HasMesh("contact_sphere"))
  {
    const common::Mesh *mesh =
      common::MeshManager::Instance()->GetMesh("contact_sphere");
    this->InsertMesh(mesh);
  }

  this->connections.push_back(
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
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->contactsMsg || !this->receivedMsg)
    return;

  // The following values are used to calculate normal scaling factor based
  // on force value.
  double magScale = 100;
  double vMax = 0.5;
  double vMin = 0.1;
  double vRange = vMax - vMin;
  double offset = vRange - vMin;

  unsigned int c = 0;
  for (int i = 0; i < this->contactsMsg->contact_size(); i++)
  {
    for (int j = 0; j < this->contactsMsg->contact(i).position_size(); j++)
    {
      math::Vector3 pos = msgs::Convert(
          this->contactsMsg->contact(i).position(j));
      math::Vector3 normal = msgs::Convert(
          this->contactsMsg->contact(i).normal(j));
      double depth = this->contactsMsg->contact(i).depth(j);

      math::Vector3 force = msgs::Convert(
          this->contactsMsg->contact(i).wrench(j).body_1_wrench().force());

      // Scaling factor for the normal line.
      // Eq in the family of Y = 1/(1+exp(-(x^2)))
      double normalScale = (2.0 * vRange) / (1 + exp
          (-force.GetSquaredLength() / magScale)) - offset;

      // Create a new contact visualization point if necessary
      if (c >= this->points.size())
        this->CreateNewPoint();

      this->points[c]->sceneNode->setVisible(true);
      this->points[c]->sceneNode->setPosition(Conversions::Convert(pos));

      this->points[c]->normal->SetPoint(1, normal*normalScale);
      this->points[c]->depth->SetPoint(1, normal*-depth*10);

      this->points[c]->normal->setMaterial("Gazebo/LightOn");
      this->points[c]->depth->setMaterial("Gazebo/LightOff");
      this->points[c]->depth->Update();
      this->points[c]->normal->Update();
      c++;
    }
  }

  for ( ; c < this->points.size(); c++)
    this->points[c]->sceneNode->setVisible(false);

  this->receivedMsg = false;
}

/////////////////////////////////////////////////
void ContactVisual::OnContact(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (this->enabled)
  {
    this->contactsMsg = _msg;
    this->receivedMsg = true;
  }
}

/////////////////////////////////////////////////
void ContactVisual::SetEnabled(bool _enabled)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->enabled = _enabled;

  if (!enabled)
  {
    this->contactsSub.reset();

    this->contactsMsg.reset();
    this->receivedMsg = false;

    for (unsigned int c = 0 ; c < this->points.size(); c++)
      this->points[c]->sceneNode->setVisible(false);
  }
  else if (!this->contactsSub)
  {
    this->contactsSub = this->node->Subscribe(this->topicName,
        &ContactVisual::OnContact, this);
  }
}

/////////////////////////////////////////////////
void ContactVisual::CreateNewPoint()
{
  std::string objName = this->GetName() +
    "_contactpoint_" + boost::lexical_cast<std::string>(this->points.size());

  /// \todo We can improve this by using instanced geometry.
  Ogre::Entity *obj = this->scene->GetManager()->createEntity(
                      objName, "contact_sphere");
  obj->setMaterialName("Gazebo/BlueLaser");

  ContactVisual::ContactPoint *cp = new ContactVisual::ContactPoint();
  cp->sceneNode = this->sceneNode->createChildSceneNode(objName + "_node");
  cp->sceneNode->attachObject(obj);

  cp->normal = new DynamicLines(RENDERING_LINE_LIST);
  cp->depth = new DynamicLines(RENDERING_LINE_LIST);

  cp->normal->AddPoint(math::Vector3(0, 0, 0));
  cp->normal->AddPoint(math::Vector3(0, 0, 0.1));

  cp->depth->AddPoint(math::Vector3(0, 0, 0));
  cp->depth->AddPoint(math::Vector3(0, 0, -1));

  obj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  cp->depth->setVisibilityFlags(GZ_VISIBILITY_GUI);
  cp->normal->setVisibilityFlags(GZ_VISIBILITY_GUI);

  cp->sceneNode->attachObject(cp->depth);
  cp->sceneNode->attachObject(cp->normal);
  cp->sceneNode->setVisible(false);

  this->points.push_back(cp);
}
