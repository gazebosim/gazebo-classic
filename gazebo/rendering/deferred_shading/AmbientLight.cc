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

#include <OgreMaterialManager.h>
#include <OgreRoot.h>
#include <OgreRenderSystem.h>

#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/deferred_shading/GeomUtils.hh"
#include "gazebo/rendering/deferred_shading/AmbientLight.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
AmbientLight::AmbientLight()
{
  this->setRenderQueueGroup(Ogre::RENDER_QUEUE_2);

  this->mRenderOp.vertexData = new Ogre::VertexData();
  this->mRenderOp.indexData = 0;

  GeomUtils::CreateQuad(mRenderOp.vertexData);

  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
  this->mRenderOp.useIndexes = false;

  // Set bounding
  this->setBoundingBox(Ogre::AxisAlignedBox(-10000, -10000, -10000,
                                             10000,  10000,  10000));
  this->radius = 15000;

  this->matPtr = Ogre::MaterialManager::getSingleton().getByName(
      "DeferredShading/AmbientLight");

  if (this->matPtr.isNull())
    gzthrow("Is Null");

  this->matPtr->load();

  // This shader needs to be aware if its running under OpenGL or DirectX.
  // Real depthFactor = (Root::getSingleton().getRenderSystem()->getName() ==
  //    "OpenGL Rendering Subsystem") ? 2.0 : 1.0;
  // this->matPtr->getTechnique(0)->getPass(
  // 0)->getFragmentProgramParameters()->setNamedConstant(
  //        "depthFactor", depthFactor);
}

/////////////////////////////////////////////////
AmbientLight::~AmbientLight()
{
  // need to release IndexData and vertexData created for renderable
  delete this->mRenderOp.indexData;
  delete this->mRenderOp.vertexData;
}

/////////////////////////////////////////////////
Ogre::Real AmbientLight::getBoundingRadius(void) const
{
  return this->radius;
}

/////////////////////////////////////////////////
Ogre::Real AmbientLight::getSquaredViewDepth(const Ogre::Camera *) const
{
  return 0.0;
}

/////////////////////////////////////////////////
const Ogre::MaterialPtr& AmbientLight::getMaterial(void) const
{
  return this->matPtr;
}

/////////////////////////////////////////////////
void AmbientLight::getWorldTransforms(Ogre::Matrix4 *_xform) const
{
  *_xform = Ogre::Matrix4::IDENTITY;
}

/////////////////////////////////////////////////
void AmbientLight::UpdateFromCamera(Ogre::Camera *_camera)
{
  Ogre::Technique* tech = this->getMaterial()->getBestTechnique();
  Ogre::Vector3 farCorner = _camera->getViewMatrix(true) *
                            _camera->getWorldSpaceCorners()[4];

  for (uint16_t i = 0; i < tech->getNumPasses(); ++i)
  {
    Ogre::Pass *pass = tech->getPass(i);

    // get the vertex shader parameters
    Ogre::GpuProgramParametersSharedPtr params =
      pass->getVertexProgramParameters();

    // set the camera's far-top-right corner
    if (params->_findNamedConstantDefinition("farCorner"))
      params->setNamedConstant("farCorner", farCorner);

    params = pass->getFragmentProgramParameters();
    if (params->_findNamedConstantDefinition("farCorner"))
      params->setNamedConstant("farCorner", farCorner);
  }
}
