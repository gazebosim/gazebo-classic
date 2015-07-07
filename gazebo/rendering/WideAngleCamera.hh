//
// Created by klokik on 02.07.15.
//

#ifndef _GAZEBO_RENDERING_WIDEANGLECAMERA_HH_
#define _GAZEBO_RENDERING_WIDEANGLECAMERA_HH_

#include "Camera.hh"


namespace gazebo
{
  namespace rendering
  {
    class GAZEBO_VISIBLE WideAngleCamera : public Camera
    {
      public: WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene, bool _autoRender = true);

      public: ~WideAngleCamera();
      //public: virtual void Load() override;

      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target) override;

      public: Ogre::CompositorInstance *wamapInstance;
    };
  }
}


#endif
