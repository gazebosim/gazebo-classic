/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "rendering/ogre.h"
#include "common/MeshManager.hh"
#include "transport/Node.hh"
#include "transport/Subscriber.hh"
#include "msgs/msgs.h"
#include "rendering/Conversions.hh"
#include "rendering/Scene.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/ContactVisual.hh"

using namespace gazebo;
using namespace rendering;

/// \brief Constructor
ContactVisual::ContactVisual (const std::string &_name, Scene *_scene, 
                              const std::string &_topicName)
 : Visual(_name, _scene), scene(_scene)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(_scene->GetName());

  this->contactsSub = this->node->Subscribe(_topicName, 
      &ContactVisual::OnContact, this);

  common::MeshManager::Instance()->CreateSphere("contact_sphere", 1.0, 5, 5);

  // Add the mesh into OGRE
  if (!this->sceneNode->getCreator()->hasEntity("contact_sphere") &&
      common::MeshManager::Instance()->HasMesh("contact_sphere"))
  {
    const common::Mesh *mesh = common::MeshManager::Instance()->GetMesh("contact_sphere");

    this->InsertMesh( mesh );
  }

  this->instancedGeom = this->scene->GetManager()->createInstancedGeometry(
      "contact_instanced_geom");

  this->instancedGeom->setBatchInstanceDimensions (Ogre::Vector3(1000000, 1000000, 1000000));
  this->instancedGeom->setCastShadows(false);

  this->obj = this->scene->GetManager()->createEntity(
      "contact bitch", "unit_sphere");
  this->SetupInstancedMaterialToEntity(this->obj);
  this->instancedGeom->addEntity(this->obj, Ogre::Vector3::ZERO);

  this->instancedGeom->setOrigin(Ogre::Vector3::ZERO);
  this->instancedGeom->build();
  this->instancedGeom->addBatchInstance();

  this->connections.push_back( 
      event::Events::ConnectPreRender( 
        boost::bind(&ContactVisual::Update, this) ) );

  this->instancedGeom->setVisible(true);
  this->scene->GetManager()->destroyEntity(this->obj);
}

ContactVisual::~ContactVisual()
{
  this->instancedGeom->reset();
  delete this->instancedGeom;
}

void ContactVisual::Update()
{
  /*
  printf("Here\n");
  Ogre::InstancedGeometry::BatchInstanceIterator regIt = this->instancedGeom->getBatchInstanceIterator();
  Ogre::InstancedGeometry::BatchInstance *r = regIt.getNext();
  Ogre::InstancedGeometry::BatchInstance::InstancedObjectIterator bit = r->getObjectIterator();
  Ogre::InstancedGeometry::InstancedObject* obj = bit.getNext();

  for (int i=0; this->contactsMsg && 
       i < this->contactsMsg->contact_size(); i++)
  {
    math::Vector3 pos = msgs::Convert(
        this->contactsMsg->contact(i).position(0));
    math::Vector3 normal = msgs::Convert(
        this->contactsMsg->contact(i).normal(0));
    double depth = this->contactsMsg->contact(i).depth(0);
    obj->setPosition(Conversions::Convert(pos));

    std::cout << pos << "\n";
  }
  */
}

void ContactVisual::OnContact(
    const boost::shared_ptr<msgs::Contacts const> &_msg)
{
  this->contactsMsg = _msg;
}

void ContactVisual::SetupInstancedMaterialToEntity(Ogre::Entity *ent)
{
  for (Ogre::uint i = 0; i < ent->getNumSubEntities(); ++i)
  {
    Ogre::SubEntity* se = ent->getSubEntity(i);
    Ogre::String materialName= se->getMaterialName();
    se->setMaterialName(this->BuildInstancedMaterial("Gazebo/Red"));
  }
}

Ogre::String ContactVisual::BuildInstancedMaterial(
    const Ogre::String &originalMaterialName)
{
  // already instanced ?
  if (originalMaterialName.find("/instanced"))
    return originalMaterialName;

  Ogre::MaterialPtr originalMaterial = 
  Ogre::MaterialManager::getSingleton().getByName (originalMaterialName);

  // if originalMat doesn't exists use "Instancing" material name
  const Ogre::String instancedMaterialName(
      originalMaterial.isNull() ? "Instancing" :
      originalMaterialName + "/Instanced");

  Ogre::MaterialPtr  instancedMaterial = 
  Ogre::MaterialManager::getSingleton().getByName (instancedMaterialName);

  // already exists ?
  if (instancedMaterial.isNull())
  {
    instancedMaterial = originalMaterial->clone(instancedMaterialName);
    instancedMaterial->load();
    Ogre::Technique::PassIterator pIt = 
      instancedMaterial->getBestTechnique ()->getPassIterator();
    while (pIt.hasMoreElements())
    {
      Ogre::Pass * const p = pIt.getNext();
      p->setVertexProgram("Instancing", false);
      p->setShadowCasterVertexProgram("InstancingShadowCaster");
    }
  }

  instancedMaterial->load();
  return instancedMaterialName;
}
