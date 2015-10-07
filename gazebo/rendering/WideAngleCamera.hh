/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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


#include <string>
#include <utility>
#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

#include "gazebo/rendering/Camera.hh"


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
    /// \brief Describes a lens of a camera
    ///   as amapping function of type r = c1*f*fun(theta/c2+c3)
    class GAZEBO_VISIBLE CameraLens
    {
      /// \brief Constructor
      public: CameraLens();

      /// \brief Destructor
      public: ~CameraLens();

      /// \brief Init custom camera lens with specified parameters
      /// \param[in] _c1 Image scaling factor
      /// \param[in] _c2 Angle scaling factor
      /// \param[in] _fun Angle transform function
      /// \param[in] _f Focal length of the optical system
      /// \param[in] _c3 Angle shift parameter, should be 0 in most cases
      public: void Init(const double _c1, const double _c2,
                        const std::string &_fun, const double _f,
                        const double _c3);

      /// \brief Init camera lens with standard mapping function
      /// \param[in] _name Mapping function name
      public: void Init(const std::string &_name);

      /// \brief Load camera lens from SDF file
      /// \param[in] _sdf SDF lens element
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Load camera lens with default parameters
      public: void Load();

      /// \brief Get lens projection type
      /// \return Lens projection type string
      public: std::string Type() const;

      /// \brief Checks if lens type is of the custom type
      /// \return True if this->Type() == "custom"
      public: bool IsCustom() const;

      /// \brief Gets c1 parameter
      /// \return c1 parameter
      public: double C1() const;

      /// \brief Gets c2 parameter
      /// \return c2 parameter
      public: double C2() const;

      /// \brief Gets c3 parameter
      /// \return c3 parameter
      public: double C3() const;

      /// \brief Gets f parameter
      /// \return f parameter
      public: double F() const;

      /// \brief Gets angle transform function
      /// \return Angle transform function string
      public: std::string Fun() const;

      /// \brief Gets cut off angle
      /// \return Cut off angle
      public: double CutOffAngle() const;

      /// \brief Checks if image should be scaled to fit horizontal FOV
      /// \return True if the image will be scaled
      public: bool ScaleToHFOV() const;

      /// \brief Set lens projection type
      /// \param[in] _type Lens projection type string
      public: void SetType(const std::string &_type);

      /// \brief Sets c1 parameter
      /// \param[in] _c c1 parameter
      public: void SetC1(const double _c);

      /// \brief Sets c2 parameter
      /// \param[in] _c c2 parameter
      public: void SetC2(const double _c);

      /// \brief Sets c3 parameter
      /// \param[in] _c c3 parameter
      public: void SetC3(const double _c);

      /// \brief Sets f parameter
      /// \param[in] _f f parameter
      public: void SetF(const double _f);

      /// \brief Sets angle transform function
      /// \param[in] _fun Angle transform function string
      public: void SetFun(const std::string &_fun);

      /// \brief Sets cut-off angle
      /// \param[in] _angle cut-off angle
      public: void SetCutOffAngle(const double _angle);

      /// \brief Sets whether the image should be scaled to fit horizontal FOV
      /// \param[in] _scale true if it should,
      ///   note: c1 and f parameters are ignored in this case
      public: void SetScaleToHFOV(const bool _scale);

      /// \brief Set uniform variables of a shader
      ///   for the provided material technique pass
      /// \param[in] _pass Ogre::Pass used for rendering
      /// \param[in] _ratio Frame aspect ratio
      /// \param[in] _hfov Horizontal field of view
      public: void SetUniformVariables(Ogre::Pass *_pass, const float _ratio,
                                       const float _hfov);

      /// \internal
      /// \brief Converts projection type from one of presets to `custom`
      private: void ConvertToCustom();

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<CameraLensPrivate> dataPtr;
    };

    /// \class WideAngleCamera WideAngleCamera.hh rendering/rendering.hh
    /// \brief Camera with variable mapping function
    class GAZEBO_VISIBLE WideAngleCamera :
        public Camera,
        protected Ogre::CompositorInstance::Listener
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      /// \param[in] _textureSize Size of cube map texture used for rendering,
      ///   may be overriten in sdf
      public: WideAngleCamera(const std::string &_namePrefix, ScenePtr _scene,
                              const bool _autoRender = true,
                              const int _textureSize = 256);

      /// \brief Destructor
      public: virtual ~WideAngleCamera();

      // Documentation inherited
      public: void Init() override;

      // Documentation inherited
      public: void Load() override;

      // Documentation inherited
      public: virtual void Fini() override;

      /// \brief Gets the environment texture size
      /// \return Texture size
      public: int EnvTextureSize() const;

      /// \brief Gets camera's lens description
      /// \return Camera's lens description
      public: CameraLens *Lens() const;

      // Documentation inherited
      public: void SetRenderTarget(Ogre::RenderTarget *_target) override;

      /// \brief Sets environment texture size
      /// \param[in] _size Texture size
      public: void SetEnvTextureSize(const int _size);

      /// \brief Creates a set of 6 cameras pointing in different directions
      protected: void CreateEnvCameras();

      // Documentation inherited
      public: void SetClipDist() override;

      /// \brief Set the camera's render target
      /// \param[in] _textureName Name used as a base for environment texture
      protected: void CreateEnvRenderTexture(const std::string &_textureName);

      // Documentation inherited
      protected: void RenderImpl() override;

      /// \bried Callback that is used to set mapping material uniform values,
      ///   implements Ogre::CompositorInstance::Listener interface
      /// \param[in] _pass_id Pass identifier
      /// \param[in] _material Material whose parameters should be set
      protected: void notifyMaterialRender(Ogre::uint32 _pass_id,
        Ogre::MaterialPtr &_material) override;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<WideAngleCameraPrivate> dataPtr;
    };
    /// \}
  }
}


#endif
