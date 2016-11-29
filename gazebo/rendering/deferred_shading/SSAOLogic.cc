/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <Ogre.h>

#include "gazebo/rendering/deferred_shading/SSAOLogic.hh"

using namespace gazebo;
using namespace rendering;

class SSAOListener: public Ogre::CompositorInstance::Listener
{
  public: explicit SSAOListener(Ogre::CompositorInstance *_instance)
          : instance(_instance) {}

  // this callback we will use to modify SSAO parameters
  public: void NotifyMaterialRender(uint32_t _passId,
                                    Ogre::MaterialPtr &_mat)
          {
            // not SSAO, return
            if (_passId != 42)
              return;

            // this is the camera you're using
            Ogre::Camera *cam =
              this->instance->getChain()->getViewport()->getCamera();

            // calculate the far-top-right corner in view-space
            Ogre::Vector3 farCorner =
              cam->getViewMatrix(true) * cam->getWorldSpaceCorners()[4];

            // get the pass
            Ogre::Pass *pass = _mat->getBestTechnique()->getPass(0);

            // get the vertex shader parameters
            Ogre::GpuProgramParametersSharedPtr params =
              pass->getVertexProgramParameters();

            // set the camera's far-top-right corner
            if (params->_findNamedConstantDefinition("farCorner"))
              params->setNamedConstant("farCorner", farCorner);

            // get the fragment shader parameters
            params = pass->getFragmentProgramParameters();

            // set the projection matrix we need
            static const Ogre::Matrix4 CLIP_SPACE_TO_IMAGE_SPACE(
                0.5,    0,    0,  0.5,
                0,   -0.5,    0,  0.5,
                0,      0,    1,    0,
                0,      0,    0,    1);

            if (params->_findNamedConstantDefinition("ptMat"))
              params->setNamedConstant("ptMat",
                  CLIP_SPACE_TO_IMAGE_SPACE *
                  cam->getProjectionMatrixWithRSDepth());

            if (params->_findNamedConstantDefinition("far"))
              params->setNamedConstant("far", cam->getFarClipDistance());
          }
  private: Ogre::CompositorInstance *instance;
};

/////////////////////////////////////////////////
Ogre::CompositorInstance::Listener *SSAOLogic::createListener(
    Ogre::CompositorInstance *_instance)
{
  return new SSAOListener(_instance);
}
