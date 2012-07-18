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
#ifndef AMBIENTLIGHT_HH
#define AMBIENTLIGHT_HH

#include <OgreSimpleRenderable.h>

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
    class AmbientLight : public Ogre::SimpleRenderable
    {
      /// \brief Constructor
      public: AmbientLight();

      /// \brief Destructor
      public: ~AmbientLight();

      /// \copydoc MovableObject::getBoundingRadius
      public: virtual Ogre::Real getBoundingRadius(void) const;

      /// \copydoc Renderable::getSquaredViewDepth
      public: virtual Ogre::Real getSquaredViewDepth(const Ogre::Camera*) const;

      /// \copydoc Renderable::getMaterial
      public: virtual const Ogre::MaterialPtr &getMaterial(void) const;

      public: virtual void getWorldTransforms(Ogre::Matrix4 *_xform) const;

      public: void UpdateFromCamera(Ogre::Camera *_camera);

      protected: Ogre::Real radius;
      protected: Ogre::MaterialPtr matPtr;
    };
  }
}
#endif
