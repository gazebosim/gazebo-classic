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
#ifndef _DEFERREDLIGHT_HH_
#define _DEFERREDLIGHT_HH_

#include <OgreSimpleRenderable.h>
#include <OgreInstancedEntity.h>
#include <OgreTechnique.h>

#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// Deferred light geometry. Each instance matches a normal light.
    /// Should not be created by the user.
    /// XXX support other types of light other than point lights.
    class GZ_RENDERING_DEFERRED_VISIBLE DeferredLight: public Ogre::SimpleRenderable
    {
      /// \brief Constructor
      public: DeferredLight(MaterialGenerator *_gen, Ogre::Light *_parentLight,
                            Ogre::MaterialPtr _vplMaterial);

      /// \brief Destructor
      public: ~DeferredLight();

      /// \brief Update the information from the light that matches this one
      public: void UpdateFromParent();

      /// \breif Update the information that is related to the camera
      public: void UpdateFromCamera(Ogre::Camera *_camera);

      /// \brief Does this light cast shadows?
      public: virtual bool getCastShadows() const;

      /// @copydoc MovableObject::getBoundingRadius
      public: virtual Ogre::Real getBoundingRadius(void) const;

      /// @copydoc Renderable::getSquaredViewDepth
      public: virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera*) const;

      /// @copydoc Renderable::getMaterial
      public: virtual const Ogre::MaterialPtr &getMaterial(void) const;

      /// @copydoc Renderable::getBoundingRadius
      public: virtual void getWorldTransforms(Ogre::Matrix4 *_xform) const;

      public: virtual void UpdateRSM(const Ogre::TexturePtr &_shadowTex);
      public: virtual void UpdateShadowInvProj(Ogre::Matrix4 &_invproj);
      public: Ogre::Light *GetParentLight() {return this->parentLight;}

      public: void SetVPLCount(uint32_t _n, Ogre::SceneManager *_sm,
                               Ogre::InstanceManager *_im);

      public: void RenderVPLs(Ogre::SceneManager *_sm,
                              Ogre::InstanceManager *_im);

      /// Check if the camera is inside a light
      protected: bool IsCameraInsideLight(Ogre::Camera *_camera);

      /// Create geometry for this light.
      protected: void RebuildGeometry(float _radius);

      /// Create a sphere geometry.
      protected: void CreateSphere(float _radius, int _nRings, int _nSegments);

      /// Create a rectangle.
      protected: void CreateRectangle2D();

      /// Create a cone.
      protected: void CreateCone(float _radius, float _height,
                                 int _nVerticesInBase);

      /// Set constant, linear, quadratic Attenuation terms
      protected: void SetAttenuation(float _c, float _b, float _a);

      /// Set the specular colour
      protected: void SetSpecularColor(const Ogre::ColourValue &_col);

      /// The light that this DLight renders
      protected: Ogre::Light *parentLight;

      /// Mode to ignore world orientation/position
      protected: bool ignoreWorld;

      /// Bounding sphere radius
      protected: float radius;

      /// Deferred shading system this minilight is part of
      protected: MaterialGenerator *generator;

      /// Material permutation
      protected: unsigned int permutation;

      protected: Ogre::MaterialPtr VPLMaterial;
    };
  }
}
#endif
