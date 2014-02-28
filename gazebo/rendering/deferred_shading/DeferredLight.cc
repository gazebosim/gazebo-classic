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
#include <OgreHardwareBufferManager.h>
#include <OgreCamera.h>
#include <OgreSceneNode.h>
#include <OgreLight.h>
#include <OgreTechnique.h>
#include <OgreSceneManager.h>

#include "gazebo/common/Console.hh"
#include "gazebo/math/Helpers.hh"

#include "gazebo/rendering/Conversions.hh"

#include "gazebo/rendering/deferred_shading/GeomUtils.hh"
#include "gazebo/rendering/deferred_shading/TechniqueDefinitions.hh"
#include "gazebo/rendering/deferred_shading/LightMaterialGenerator.hh"
#include "gazebo/rendering/deferred_shading/DeferredLight.hh"

#define ENABLE_BIT(mask, flag) (mask) |= (flag)
#define DISABLE_BIT(mask, flag) (mask) &= ~(flag)

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
DeferredLight::DeferredLight(MaterialGenerator *_sys,
                             Ogre::Light *_parentLight,
                             Ogre::MaterialPtr _vplMaterial)
  : parentLight(_parentLight), ignoreWorld(false), generator(_sys),
    permutation(0), VPLMaterial(_vplMaterial)
{
  // Set up geometry
  // Allocate render operation
  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->mRenderOp.indexData = 0;
  this->mRenderOp.vertexData = 0;
  this->mRenderOp.useIndexes = true;

  this->VPLMaterial->load();
  this->UpdateFromParent();
}

/////////////////////////////////////////////////
DeferredLight::~DeferredLight()
{
  // need to release IndexData and vertexData created for renderable
  delete this->mRenderOp.indexData;
  delete this->mRenderOp.vertexData;
}

/////////////////////////////////////////////////
void DeferredLight::SetAttenuation(float _c, float _b, float _a)
{
  // Set Attenuation parameter to shader
  setCustomParameter(3, Ogre::Vector4(_c, _b, _a, 0));
  float outerRadius = this->parentLight->getAttenuationRange();

  /// There is attenuation? Set material accordingly
  if (!math::equal(_c, 1.0f) || !math::equal(_b, 0.0f) ||
      !math::equal(_a, 0.0f))
  {
    ENABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_ATTENUATED);

    if (this->parentLight->getType() == Ogre::Light::LT_POINT)
    {
      /// Calculate radius from Attenuation
      // difference of 10-15 levels deemed unnoticeable
      int threshold_level = 10;
      float threshold = 1.0f / (static_cast<float>(threshold_level)/256.0f);

      /// Use quadratic formula to determine outer radius
      _c = _c - threshold;
      float d = sqrt(_b * _b - 4 * _a * _c);
      outerRadius = (-2 * _c) / (_b + d);
      outerRadius *= 1.2;
    }
  }
  else
  {
    DISABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_ATTENUATED);
  }

  this->RebuildGeometry(outerRadius);
}

/////////////////////////////////////////////////
void DeferredLight::SetSpecularColor(const Ogre::ColourValue &_col)
{
  // setCustomParameter(2, Vector4(col.r, col.g, col.b, col.a));
  // There is a specular component? Set material accordingly

  if (!math::equal(_col.r, 0.0f) || !math::equal(_col.g, 0.0f) ||
      !math::equal(_col.b, 0.0f))
  {
    ENABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_SPECULAR);
  }
  else
  {
    DISABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_SPECULAR);
  }
}

/////////////////////////////////////////////////
void DeferredLight::RebuildGeometry(float _radius)
{
  // Disable all 3 bits
  DISABLE_BIT(this->permutation,
      LightMaterialGenerator<NullTechnique>::MI_POINT);
  DISABLE_BIT(this->permutation,
      LightMaterialGenerator<NullTechnique>::MI_SPOTLIGHT);
  DISABLE_BIT(this->permutation,
      LightMaterialGenerator<NullTechnique>::MI_DIRECTIONAL);

  switch (this->parentLight->getType())
  {
    case Ogre::Light::LT_DIRECTIONAL:
      {
        this->CreateRectangle2D();
        ENABLE_BIT(this->permutation,
            LightMaterialGenerator<NullTechnique>::MI_DIRECTIONAL);
        break;
      }
    case Ogre::Light::LT_POINT:
      {
        /// XXX some more intelligent expression for rings and segments
        this->CreateSphere(_radius, 10, 10);
        ENABLE_BIT(this->permutation,
            LightMaterialGenerator<NullTechnique>::MI_POINT);
        break;
      }
    case Ogre::Light::LT_SPOTLIGHT:
      {
        Ogre::Real height, rad;
        Ogre::Radian coneRadiusAngle;

        height = this->parentLight->getAttenuationRange();
        coneRadiusAngle = this->parentLight->getSpotlightOuterAngle() / 2.0;

        rad = Ogre::Math::Tan(coneRadiusAngle) * height;
        this->CreateCone(rad, height, 20);
        ENABLE_BIT(this->permutation,
            LightMaterialGenerator<NullTechnique>::MI_SPOTLIGHT);
        break;
      }
    default:
      gzerr << "Shouldn't get here\n";
  };
}

/////////////////////////////////////////////////
void DeferredLight::CreateRectangle2D()
{
  /// XXX this RenderOp should really be re-used between DeferredLight objects,
  /// not generated every time
  delete this->mRenderOp.vertexData;
  delete this->mRenderOp.indexData;

  this->mRenderOp.vertexData = new Ogre::VertexData();
  this->mRenderOp.indexData = 0;

  GeomUtils::CreateQuad(this->mRenderOp.vertexData);

  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
  this->mRenderOp.useIndexes = false;

  // Set bounding
  this->setBoundingBox(Ogre::AxisAlignedBox(
        -10000, -10000, -10000, 10000, 10000, 10000));

  this->radius = 15000;
  this->ignoreWorld = true;
}

/////////////////////////////////////////////////
void DeferredLight::CreateSphere(float _radius, int _nRings, int _nSegments)
{
  delete this->mRenderOp.vertexData;
  delete this->mRenderOp.indexData;

  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->mRenderOp.indexData = new Ogre::IndexData();
  this->mRenderOp.vertexData = new Ogre::VertexData();
  this->mRenderOp.useIndexes = true;

  GeomUtils::CreateSphere(this->mRenderOp.vertexData,
      this->mRenderOp.indexData , _radius , _nRings, _nSegments,
      false, false);

  // Give a little bit of padding
  _radius *= 1.1;

  // Set bounding box and sphere
  this->setBoundingBox(Ogre::AxisAlignedBox(
        Ogre::Vector3(-_radius, -_radius, -_radius),
        Ogre::Vector3(radius, radius, radius)));

  this->radius = _radius;
  this->ignoreWorld = false;
}

/////////////////////////////////////////////////
void DeferredLight::CreateCone(float _radius, float _height,
                               int _nVerticesInBase)
{
  delete this->mRenderOp.vertexData;
  delete this->mRenderOp.indexData;

  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->mRenderOp.indexData = new Ogre::IndexData();
  this->mRenderOp.vertexData = new Ogre::VertexData();
  this->mRenderOp.useIndexes = true;

  GeomUtils::CreateCone(this->mRenderOp.vertexData,
      this->mRenderOp.indexData , _radius , _height, _nVerticesInBase);

  // Set bounding box and sphere
  this->setBoundingBox(Ogre::AxisAlignedBox(
        Ogre::Vector3(-_radius, 0, -_radius),
        Ogre::Vector3(_radius, _height, _radius)));

  this->radius = _radius;
  this->ignoreWorld = false;
}

/////////////////////////////////////////////////
Ogre::Real DeferredLight::getBoundingRadius(void) const
{
  return this->radius;
}

/////////////////////////////////////////////////
Ogre::Real DeferredLight::getSquaredViewDepth(const Ogre::Camera *_cam) const
{
  if (this->ignoreWorld)
    return 0.0f;
  else
  {
    Ogre::Vector3 dist = _cam->getDerivedPosition() -
      this->getParentSceneNode()->_getDerivedPosition();

    return dist.squaredLength();
  }
}

/////////////////////////////////////////////////
const Ogre::MaterialPtr &DeferredLight::getMaterial(void) const
{
  return this->generator->GetMaterial(this->permutation);
}

/////////////////////////////////////////////////
void DeferredLight::getWorldTransforms(Ogre::Matrix4 *_xform) const
{
  if (this->parentLight->getType() == Ogre::Light::LT_SPOTLIGHT)
  {
    Ogre::Quaternion quat = Ogre::Vector3::UNIT_Y.getRotationTo(
          this->parentLight->getDerivedDirection());
    _xform->makeTransform(this->parentLight->getDerivedPosition(),
        Ogre::Vector3::UNIT_SCALE, quat);
  }
  else
  {
    _xform->makeTransform(this->parentLight->getDerivedPosition(),
        Ogre::Vector3::UNIT_SCALE, Ogre::Quaternion::IDENTITY);
  }
}

/////////////////////////////////////////////////
void DeferredLight::UpdateFromParent()
{
  // TODO : Don't do this unless something changed
  this->SetAttenuation(this->parentLight->getAttenuationConstant(),
                       this->parentLight->getAttenuationLinear(),
                       this->parentLight->getAttenuationQuadric());

  this->SetSpecularColor(this->parentLight->getSpecularColour());

  if (this->getCastShadows())
    ENABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_SHADOW_CASTER);
  else
    DISABLE_BIT(this->permutation,
        LightMaterialGenerator<NullTechnique>::MI_SHADOW_CASTER);
}

/////////////////////////////////////////////////
bool DeferredLight::IsCameraInsideLight(Ogre::Camera *_camera)
{
  switch (this->parentLight->getType())
  {
    case Ogre::Light::LT_DIRECTIONAL:
      return false;
    case Ogre::Light::LT_POINT:
      {
        Ogre::Real distanceFromLight =
          _camera->getDerivedPosition().distance(
              this->parentLight->getDerivedPosition());

        // Small epsilon fix to account for the fact that we aren't a
        // true sphere.
        return distanceFromLight <= this->radius +
           _camera->getNearClipDistance() + 0.1;
      }
    case Ogre::Light::LT_SPOTLIGHT:
      {
        Ogre::Vector3 lightPos = this->parentLight->getDerivedPosition();
        Ogre::Vector3 lightDir = this->parentLight->getDerivedDirection();
        Ogre::Radian attAngle = this->parentLight->getSpotlightOuterAngle();

        // Extend the analytic cone's radius by the near clip range by
        // moving its tip accordingly.
        // Some trigonometry needed here.
        Ogre::Vector3 clipRangeFix = -lightDir *
          (_camera->getNearClipDistance() / Ogre::Math::Tan(attAngle/2.0));
        lightPos = lightPos + clipRangeFix;

        Ogre::Vector3 lightToCamDir = _camera->getDerivedPosition() - lightPos;
        Ogre::Real distanceFromLight = lightToCamDir.normalise();

        Ogre::Real cosAngle = lightToCamDir.dotProduct(lightDir);
        Ogre::Radian angle = Ogre::Math::ACos(cosAngle);

        // Check whether we will see the cone from our current POV.
        return (distanceFromLight <= (this->parentLight->getAttenuationRange()
              + clipRangeFix.length())) && (angle <= attAngle);
      }
    default:
      return false;
  }
}

/////////////////////////////////////////////////
bool DeferredLight::getCastShadows() const
{
  return this->parentLight->_getManager()->isShadowTechniqueInUse() &&
         this->parentLight->getCastShadows() &&
         (this->parentLight->getType() == Ogre::Light::LT_DIRECTIONAL ||
          this->parentLight->getType() == Ogre::Light::LT_SPOTLIGHT);
}

/////////////////////////////////////////////////
void DeferredLight::UpdateFromCamera(Ogre::Camera *_camera)
{
  // Set shader params
  const Ogre::MaterialPtr &mat = this->getMaterial();
  if (!mat->isLoaded())
    mat->load();

  Ogre::Technique* tech = mat->getBestTechnique();
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
    {
      params->setNamedConstant("farCorner", farCorner);
    }

    this->VPLMaterial->getBestTechnique()->getPass(0)->
      getFragmentProgramParameters()->setNamedConstant("farCorner", farCorner);

    params = pass->getFragmentProgramParameters();
    if (params->_findNamedConstantDefinition("farCorner"))
      params->setNamedConstant("farCorner", farCorner);

    // If inside light geometry, render back faces with CMPF_GREATER,
    // otherwise normally
    if (this->parentLight->getType() == Ogre::Light::LT_DIRECTIONAL)
    {
      pass->setCullingMode(Ogre::CULL_CLOCKWISE);
      pass->setDepthCheckEnabled(false);
    }
    else
    {
      pass->setDepthCheckEnabled(true);
      if (this->IsCameraInsideLight(_camera))
      {
        pass->setDepthFunction(Ogre::CMPF_GREATER_EQUAL);
        pass->setCullingMode(Ogre::CULL_ANTICLOCKWISE);
      }
      else
      {
        pass->setCullingMode(Ogre::CULL_CLOCKWISE);
        pass->setDepthFunction(Ogre::CMPF_LESS_EQUAL);
      }
    }

    Ogre::Camera shadowCam("ShadowCameraSetupCam", 0);
    shadowCam._notifyViewport(_camera->getViewport());

    Ogre::SceneManager *sm = this->parentLight->_getManager();
    sm->getShadowCameraSetup()->getShadowCamera(sm,
        _camera, _camera->getViewport(), this->parentLight, &shadowCam, 0);

    // Get the shadow camera position
    if (params->_findNamedConstantDefinition("shadowCamPos"))
      params->setNamedConstant("shadowCamPos", shadowCam.getPosition());

    if (params->_findNamedConstantDefinition("shadowFarClip"))
      params->setNamedConstant("shadowFarClip", shadowCam.getFarClipDistance());
  }
}

/////////////////////////////////////////////////
void DeferredLight::UpdateRSM(const Ogre::TexturePtr &_shadowTex)
{
  this->VPLMaterial->getBestTechnique()->getPass(0)->
    getTextureUnitState("RSM")->setTexture(_shadowTex);
}

/////////////////////////////////////////////////
void DeferredLight::UpdateShadowInvProj(Ogre::Matrix4 &_invproj)
{
  this->VPLMaterial->getBestTechnique()->getPass(0)->
    getVertexProgramParameters()->setNamedConstant("InvShadowProjMatrix",
        _invproj);
}

/////////////////////////////////////////////////
void DeferredLight::SetVPLCount(uint32_t /*_n*/,
    Ogre::SceneManager * /*_sm*/, Ogre::InstanceManager * /*_im*/)
{
}

/////////////////////////////////////////////////
void DeferredLight::RenderVPLs(Ogre::SceneManager *_sm,
                               Ogre::InstanceManager *_im)
{
  Ogre::InstanceManager::InstanceBatchIterator it =
    _im->getInstanceBatchIterator(this->VPLMaterial->getName());

  _sm->_injectRenderWithPass(this->VPLMaterial->getBestTechnique()->getPass(0),
      (*it.current()), false);
}

