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
      public: WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene,
                              bool _autoRender = true, int textureSize = 256);

      public: ~WideAngleCamera();
      //public: virtual void Load() override;

      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target) override;

      public: void CreateEnvRenderTexture(const std::string &_textureName);

      private: void CreateEnvCameras();

      public: virtual void SetClipDist();

      protected: virtual void RenderImpl();

      public: virtual void Init();

      public: virtual void Load();

      public: virtual void Fini();

      protected: Ogre::CompositorInstance *wamapInstance;

      private: Ogre::Camera *envCameras[6];

      private: Ogre::RenderTarget *envRenderTargets[6];

      private: Ogre::Viewport *envViewports[6];

      private: Ogre::Texture *envCubeMapTexture;

      private: int envTextureSize;

      private: Ogre::MaterialPtr compMat;

      private: int projectionType;

      private: double c1;
      private: double c2;
      private: double c3;
      private: double f;

      private: std::string fun;
      private: math::Vector2i fun_b;
    };
  }
}


#endif
