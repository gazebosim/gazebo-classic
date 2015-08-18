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
      /// \param[in] _fun Focal length of the optical system
      /// \param[in] _c3 Angle shift constant, should be 0 in most cases
      public: void Init(float _c1, float _c2, std::string _fun, float _f, float _c3);

      /// \brief Init camera lens with standard mapping function
      /// \param[in] _name Mapping function name
      public: void Init(std::string _name);

      /// \brief Load camera lens from SDF file
      /// \param[in] _sdf SDF lens element
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load camera lens with default parameters
      public: void Load();

      /// \brief Get lens projection type
      /// \return Lens projection type string
      public: std::string GetType() const;

      /// \brief Checks if lens type is of the custom type
      /// \return True if this->GetType() == "custom"
      public: bool IsCustom() const;

      /// \brief Gets c1 constant
      /// \return c1 constant
      public: float GetC1() const;

      /// \brief Gets c2 constant
      /// \return c2 constant
      public: float GetC2() const;

      /// \brief Gets c3 constant
      /// \return c3 constant
      public: float GetC3() const;

      /// \brief Gets f constant
      /// \return f constant
      public: float GetF() const;

      /// \brief Gets angle transform function
      /// \return Angle transform function string
      public: std::string GetFun() const;

      /// \brief Gets cut off angle
      /// \return Cut off angle
      public: float GetCutOffAngle() const;

      /// \brief Checks if image should be scaled to fit horisontal FOV
      /// \return True if the image will be scaled
      public: bool GetScaleToHFOV() const;

      /// \brief Set lens projection type
      /// \param[in] _type Lens projection type string
      public: void SetType(std::string _type);

      /// \brief Sets c1 constant
      /// \param[in] _c c1 constant
      public: void SetC1(float _c);

      /// \brief Sets c2 constant
      /// \param[in] _c c2 constant
      public: void SetC2(float _c);

      /// \brief Sets c3 constant
      /// \param[in] _c c3 constant
      public: void SetC3(float _c);

      /// \brief Sets f constant
      /// \param[in] _f f constant
      public: void SetF(float _f);

      /// \brief Sets angle transform function
      /// \param[in] _fun Angle transform function string
      public: void SetFun(std::string _fun);

      /// \brief Sets cut-off angle
      /// \param[in] _angle cut-off angle
      public: void SetCutOffAngle(float _angle);

      /// \brief Sets whether the image should be scaled to fit horisontal FOV
      /// \param[in] _scale true if it should, note: c1 and f constants are ignored in this case
      public: void SetScaleToHFOV(bool _scale);

      /// \brief Set uniform variables of shader for the provided material technique pass
      /// \param[in] _pass Ogre::Pass used for rendering
      /// \param[in] _ratio Frame aspect ratio
      /// \param[in] _hfov Horisontal field of view
      public: void SetUniformVariables(Ogre::Pass *_pass, float _ratio, float _hfov);

      /// \brief Converts projection type from one of the predefined projection typed to custom
      private: void ConvertToCustom();

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

      /// \brief Initializes the camera
      public: void Init() override;

      /// \brief Loads camera with default parmeters
      public: void Load() override;

      /// \brief Finalize the camera.
      ///
      /// This function is called before the camera is destructed
      public: virtual void Fini() override;

            /// \brief Gets the environment texture size
      /// \return Texture size
      public: int GetEnvTextureSize() const;

      /// \brief Gets camera's lens description
      /// \return Camera's lens description
      public: CameraLens *GetLens();

      /// \brief Set the camera's render target
      /// \param[in] _target Pointer to the render target
      public: void SetRenderTarget(Ogre::RenderTarget *_target) override;

      /// \brief Sets environment texture size
      /// \param[in] _size Texture size
      public: void SetEnvTextureSize(int _size);

      /// \brief Creates a set of 6 cameras pointing in different directions
      protected: void CreateEnvCameras();

      /// \brief Sets the clip distance based on stored SDF values
      public: void SetClipDist() override;

      /// \brief Set the camera's render target
      /// \param[in] _textureName Name used as a base for environment texture
      protected: void CreateEnvRenderTexture(const std::string &_textureName);

      /// \brief Implementation of the render call
      protected: void RenderImpl() override;

      /// \bried Callback that is used to set mapping material uniform values, implements Ogre::CompositorInstance::Listener interface
      /// \param[in] _pass_id Pass identifier
      /// \param[in] _pass_id Material whose parameters should be set
      protected: void notifyMaterialRender(Ogre::uint32 _pass_id, Ogre::MaterialPtr &_material) override;

      /// \brief Compositor used to render rectangle with attached cube map texture
      protected: Ogre::CompositorInstance *cubeMapCompInstance;

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

      /// \brief Private data pointer
      private: WideAngleCameraPrivate *dataPtr;
    };
    /// \}
  }
}


#endif
