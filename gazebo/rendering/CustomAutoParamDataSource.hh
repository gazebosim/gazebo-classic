/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_RENDERING_CUSTOMAUTOPARAMDATASOURCE_HH_
#define GAZEBO_RENDERING_CUSTOMAUTOPARAMDATASOURCE_HH_

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Class that replaces arrays of size 8 in the original class with
    /// std::vectors that can grow as needed. This lets Ogre publish shader
    /// uniforms for more than 8 shadow maps.
    class GAZEBO_VISIBLE CustomAutoParamDataSource : public Ogre::AutoParamDataSource
    {
    public:
      CustomAutoParamDataSource() {}
      virtual ~CustomAutoParamDataSource() {}

      /// \brief Updates the current renderable
      virtual void setCurrentRenderable(const Ogre::Renderable* rend) override;

      /// \brief Sets the light list that should be used, and it's base index from the global list */
      virtual void setCurrentLightList(const Ogre::LightList* ll) override;

      /// \brief Sets the current texture projector for a index
      virtual void setTextureProjector(const Ogre::Frustum* frust, size_t index) override;

      virtual const Ogre::Matrix4& getTextureViewProjMatrix(size_t index) const override;
      virtual const Ogre::Matrix4& getTextureWorldViewProjMatrix(size_t index) const override;
      virtual const Ogre::Matrix4& getSpotlightViewProjMatrix(size_t index) const override;
      virtual const Ogre::Matrix4& getSpotlightWorldViewProjMatrix(size_t index) const override;
      virtual const Ogre::Vector4& getShadowSceneDepthRange(size_t index) const override;

    protected:
      virtual void resizeVectorsIfNecessary(size_t size) const;

      // A replacement for everything in the parent class that was a static
      // array of size OGRE_MAX_SIMULTANEOUS_LIGHTS.
      mutable std::vector<Ogre::Matrix4> mTextureViewProjMatrixVector;
      mutable std::vector<Ogre::Matrix4> mTextureWorldViewProjMatrixVector;
      mutable std::vector<Ogre::Matrix4> mSpotlightViewProjMatrixVector;
      mutable std::vector<Ogre::Matrix4> mSpotlightWorldViewProjMatrixVector;
      mutable std::vector<Ogre::Vector4> mShadowCamDepthRangesVector;
      mutable std::vector<bool> mTextureViewProjMatrixDirtyVector;
      mutable std::vector<bool> mTextureWorldViewProjMatrixDirtyVector;
      mutable std::vector<bool> mSpotlightViewProjMatrixDirtyVector;
      mutable std::vector<bool> mSpotlightWorldViewProjMatrixDirtyVector;
      mutable std::vector<bool> mShadowCamDepthRangesDirtyVector;
      mutable std::vector<const Ogre::Frustum*> mCurrentTextureProjectorVector;
    };
  }
}

#endif

