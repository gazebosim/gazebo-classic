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

#ifndef _GAZEBO_RENDERING_WIDEANGLECAMERA_HH_
#define _GAZEBO_RENDERING_WIDEANGLECAMERA_HH_

#include "Camera.hh"


namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    // forward declarations
    class CameraLensPrivate;

    /// \class CameraLens WideAngleCamera.hh rendering/rendering.hh
    /// \brief Describes a lens of a camera as amapping function of type r = c1*f*fun(theta/c2+c3)
    class GAZEBO_VISIBLE CameraLens
    {
      public: CameraLens();
      public: ~CameraLens();

      public: void Init(float c1,float c2,std::string fun,float f=1.0f,float c3=0.0f);
      public: void Init(std::string name);

      public: void Load(sdf::ElementPtr sdf);
      public: void Load();

      public: float GetC1() const;
      public: float GetC2() const;
      public: float GetC3() const;
      public: float GetF() const;
      public: std::string GetFun() const;
      public: float GetCutOffAngle() const;

      public: void SetC1(float c);
      public: void SetC2(float c);
      public: void SetC3(float c);
      public: void SetF(float f);
      public: void SetFun(std::string fun);
      public: void SetCutOffAngle(float _angle);
      public: void SetCircular(bool _circular);

      private: void ConvertToCustom();

      public: std::string GetType() const;

      public: void SetType(std::string type);

      public: bool IsCustom() const;

      public: bool IsCircular() const;

      public: void SetCompositorMaterial(Ogre::MaterialPtr material);

      public: void SetMaterialVariables(float _ratio,float _hfov);

      protected: sdf::ElementPtr sdf;

      private: Ogre::MaterialPtr compositorMaterial;

      private: CameraLensPrivate *dataPtr;
    };

    /// \class WideAngleCamera WideAngleCamera.hh rendering/rendering.hh
    /// \brief Camera with agile mapping function
    class GAZEBO_VISIBLE WideAngleCamera : public Camera
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      /// \param[in] _textureSize Size of cube map texture used for rendering, may be overriten in sdf
      public: WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene,
                              bool _autoRender = true, int textureSize = 256);

      /// \brief Destructor
      public: virtual ~WideAngleCamera();

      /// \brief Set the camera's render target
      /// \param[in] _target Pointer to the render target
      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target) override;

      /// \brief Set the camera's render target
      /// \param[in] _textureName Name used as a base for environment texture
      protected: void CreateEnvRenderTexture(const std::string &_textureName);

      /// \brief Get the environment texture size
      /// \return Texture size
      public: int GetEnvTextureSize() const;

      /// \brief Set environment texture size
      /// \param[in] _size Texture size
      public: void SetEnvTextureSize(int _size);

      /// \brief Create a set of 6 cameras pointing in different directions
      protected: void CreateEnvCameras();

      /// \brief Set the clip distance based on stored SDF values
      public: virtual void SetClipDist() override;

      /// \brief Implementation of the render call
      protected: virtual void RenderImpl() override;

      /// \brief Initialize the camera
      public: virtual void Init() override;

      /// \brief Load the camera with default parmeters
      public: virtual void Load() override;

      /// \brief Finalize the camera.
      ///
      /// This function is called before the camera is destructed
      public: virtual void Fini() override;

      /// \brief Get this camera lens description
      /// \return Camera lens description
      public: const CameraLens *GetLens();

      /// \brief Compositor used to render rectangle with attached cube map texture
      protected: Ogre::CompositorInstance *wamapInstance;

      /// \brief Set of 6 cameras, each pointing in different direction with FOV of 90deg
      protected: Ogre::Camera *envCameras[6];

      /// \brief Render targets for envCameras
      protected: Ogre::RenderTarget *envRenderTargets[6];

      /// \brief Viewports for the render targets
      protected: Ogre::Viewport *envViewports[6];

      /// \brief A single cube map texture
      protected: Ogre::Texture *envCubeMapTexture;

      /// \brief Environment texture size
      protected: int envTextureSize;

      /// \brief Pointer to material, that is assigned to 'wamapInstance' compositor
      protected: Ogre::MaterialPtr compMat;

      /// \brief Camera lens description
      protected: CameraLens lens;
    };
    /// \}
  }
}


#endif
