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
    class WideAngleCamera;
    class CameraLensPrivate;
    class WideAngleCameraPrivate;

    /// \class CameraLens WideAngleCamera.hh rendering/rendering.hh
    /// \brief Describes a lens of a camera as amapping function of type r = c1*f*fun(theta/c2+c3)
    class GAZEBO_VISIBLE CameraLens
    {
      /// \brief Constructor
      public: CameraLens();

      /// \brief Destructor
      public: ~CameraLens();

      /// \brief Init custom camera lens with specified parameters
      /// \param[in] _c1 Image scaling constant
      /// \param[in] _c2 Angle scaling constant
      /// \param[in] _fun Angle transform function
      /// \param[in] _fun Focal length of the optical system, default value is 1
      /// \param[in] _c3 Angle shift constant, default value is 0, you probably don't wont to change it
      public: void Init(float _c1,float _c2,std::string _fun,float _f=1.0f,float _c3=0.0f);

      /// \brief Init camera lens with standard mapping function
      /// \param[in] _name Mapping function name
      public: void Init(std::string _name);

      /// \brief Load camera lens from SDF file
      /// \param[in] _sdf SDF lens element
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load camera lens with default parameters
      public: void Load();

      /// \brief Get c1 constant
      /// \return c1 constant
      public: float GetC1() const;

      /// \brief Get c2 constant
      /// \return c2 constant
      public: float GetC2() const;

      /// \brief Get c3 constant
      /// \return c3 constant
      public: float GetC3() const;

      /// \brief Get f constant
      /// \return f constant
      public: float GetF() const;

      /// \brief Get angle transform function
      /// \return angle transform function string
      public: std::string GetFun() const;

      /// \brief Get cut off angle
      /// \return Cut off angle
      public: float GetCutOffAngle() const;

      /// \brief Set c1 constant
      /// \param[in] _c c1 constant
      public: void SetC1(float _c);

      /// \brief Set c2 constant
      /// \param[in] _c c2 constant
      public: void SetC2(float _c);

      /// \brief Set c3 constant
      /// \param[in] _c c3 constant
      public: void SetC3(float _c);

      /// \brief Set f constant
      /// \param[in] _f f constant
      public: void SetF(float _f);

      /// \brief Set angle transform function
      /// \param[in] _fun angle transform function string
      public: void SetFun(std::string _fun);

      /// \brief Set cut-off angle
      /// \param[in] _angle cut-off angle
      public: void SetCutOffAngle(float _angle);

      /// \brief Set whether the lens is circular
      /// \param[in] _circular Whether the lens should be circular
      public: void SetCircular(bool _circular);

      /// \brief Convert standard projection to custom
      private: void ConvertToCustom();

      /// \brief Get lens projection type
      /// \return Lens projection type string
      public: std::string GetType() const;

      /// \brief Set lens projection type
      /// \param[in] _type Lens projection type string
      public: void SetType(std::string _type);

      /// \brief Check if lens type is custom
      /// \return True if this->GetType() == "custom"
      public: bool IsCustom() const;

      /// \brief Check if lens is circular
      /// \return True if the lens is circular
      public: bool IsCircular() const;

      /// \brief Set uniform variables of shader for the provided material technique pass
      /// \param[in] _pass Ogre::Pass used for rendering
      /// \param[in] _ratio Frame aspect ratio
      /// \param[in] _hfov Horisontal field of view
      public: void SetUniformVariables(Ogre::Pass *_pass,float _ratio,float _hfov);

      /// \brief SDF element of the lens
      protected: sdf::ElementPtr sdf;

      /// \brief Private data pointer
      private: CameraLensPrivate *dataPtr;
    };

    /// \class WideAngleCamera WideAngleCamera.hh rendering/rendering.hh
    /// \brief Camera with agile mapping function
    class GAZEBO_VISIBLE WideAngleCamera : public Camera, protected Ogre::CompositorInstance::Listener
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      /// \param[in] _textureSize Size of cube map texture used for rendering, may be overriten in sdf
      public: WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene,
                              bool _autoRender = true, int _textureSize = 256);

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
      public: CameraLens *GetLens();

      protected: virtual void notifyMaterialRender(Ogre::uint32 _pass_id, Ogre::MaterialPtr &_material) override;

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
      protected: CameraLens *lens;

      private: WideAngleCameraPrivate *dataPtr;
    };
    /// \}
  }
}


#endif
