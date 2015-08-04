/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _AMBIENTLIGHT_HH_
#define _AMBIENTLIGHT_HH_

#include <OgreSimpleRenderable.h>

#include "gazebo/rendering/deferred_shading/GeomUtils.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    // Renderable for rendering Ambient component and also to
    // establish the depths
    // Just instantiation is sufficient
    // Note that instantiation is necessary to at least establish the depths
    // even if the current ambient colour is 0
    // its ambient colour is same as the scene's ambient colour
    // XXX Could make this a singleton/make it private to the
    // DeferredShadingSystem e.g.
    template<class techniquePolicy>
    class GZ_RENDERING_VISIBLE AmbientLight
      : public Ogre::SimpleRenderable, public techniquePolicy
    {
      /// \brief Constructor
      public: AmbientLight()
      {
        this->setRenderQueueGroup(Ogre::RENDER_QUEUE_2);

        this->mRenderOp.vertexData = new Ogre::VertexData();
        this->mRenderOp.indexData = 0;

        GeomUtils::CreateQuad(mRenderOp.vertexData);

        this->mRenderOp.operationType =
          Ogre::RenderOperation::OT_TRIANGLE_STRIP;
        this->mRenderOp.useIndexes = false;

        // Set bounding
        this->setBoundingBox(Ogre::AxisAlignedBox(-10000, -10000, -10000,
                                                   10000,  10000,  10000));
        this->radius = 15000;

        this->matPtr = Ogre::MaterialManager::getSingleton().getByName(
            this->GetMaterialPrefix() + "/AmbientLight");

        if (this->matPtr.isNull())
          gzthrow("Is Null");

        this->matPtr->load();
      }

      /// \brief Destructor
      public: ~AmbientLight()
      {
        // need to release IndexData and vertexData created for renderable
        delete this->mRenderOp.indexData;
        delete this->mRenderOp.vertexData;
      }

      /// \copydoc MovableObject::getBoundingRadius
      public: virtual Ogre::Real getBoundingRadius(void) const
      {
        return this->radius;
      }

      /// \copydoc Renderable::getSquaredViewDepth
      public: virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera*) const
      {
        return 0.0;
      }

      /// \copydoc Renderable::getMaterial
      public: virtual const Ogre::MaterialPtr &getMaterial(void) const
      {
        return this->matPtr;
      }

      public: virtual void getWorldTransforms(Ogre::Matrix4 *_xform) const
      {
        *_xform = Ogre::Matrix4::IDENTITY;
      }

      public: void UpdateFromCamera(Ogre::Camera *_camera)
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

      protected: Ogre::Real radius;
      protected: Ogre::MaterialPtr matPtr;
    };
  }
}
#endif
