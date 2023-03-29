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

#ifndef GAZEBO_RENDERING_LENSFLARE_HH_
#define GAZEBO_RENDERING_LENSFLARE_HH_

#include <memory>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    class LensFlarePrivate;

    class LensFlareCompositorListenerPrivate;

    /// \brief We'll create an instance of this class for each camera, to be
    /// used to inject lens flare uniforms in each render call.
    class GZ_RENDERING_VISIBLE LensFlareCompositorListener
      : public Ogre::CompositorInstance::Listener
      {
      /// \brief Constructor
      public: LensFlareCompositorListener(CameraPtr _camera, LightPtr _light);

      /// \brief Destructor
      public: ~LensFlareCompositorListener();

      /// \brief Set light that generates lens flare
      /// \param[in] _light Pointer to light
      public: void SetLight(LightPtr _light);

      /// \brief Set the scale of lens flare.
      /// \param[in] _scale Scale of lens flare
      public: void SetScale(const double _scale);

      /// \brief Set the color of lens flare.
      /// \param[in] _color Color of lens flare
      public: void SetColor(const ignition::math::Vector3d &_color);

      /// \brief Set the number of steps to take in each direction when
      /// checking for occlusions.
      /// \param[in] _occlusionSteps number of steps to take in each direction
      /// when checking for occlusion.
      public: void SetOcclusionSteps(double _occlusionSteps);

      /// \brief Callback that OGRE will invoke for us on each render call
      /// \param[in] _passID OGRE material pass ID.
      /// \param[in] _mat Pointer to OGRE material.
      public: virtual void notifyMaterialRender(unsigned int _passId,
                                                Ogre::MaterialPtr &_mat);

      /// \brief Get the lens flare position and scale for a normal camera
      /// \param[in] _camera Camera which the lens flare is added to
      /// \param[out] _pos lens flare position in normalized device coordinates
      /// \param[out] _scale Amount to scale the lens flare by.
      private: void CameraPosScale(const CameraPtr &_camera,
          ignition::math::Vector3d &_pos, double &_scale);

      /// \brief Get the lens flare position and scale for a wide angle camera
      /// \param[in] _wideAngleCam Camera which the lens flare is added to
      /// \param[out] _pos lens flare position in normalized device coordinates
      /// \param[out] _scale Amount to scale the lens flare by.
      private: void WideAngleCameraPosScale(
          const WideAngleCameraPtr &_wideAngleCam,
          ignition::math::Vector3d &_pos, double &_scale);

      /// \brief Check to see if the lens flare is occluded and return a scaling
      /// factor that is proportional to the lens flare's visibility
      /// \param[in] _cam Camera used for checking occlusion
      /// \param[in] _imgPos light pos in clip space
      /// \param[in] _worldPos light pos in 3D world space
      private: double OcclusionScale(const CameraPtr &_cam,
                                     const ignition::math::Vector3d &_imgPos,
                                     const ignition::math::Vector3d &_worldPos);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LensFlareCompositorListenerPrivate> dataPtr;
    };

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class LensFlare LensFlare.hh rendering/rendering.hh
    /// \brief Camera lens flare compositor. This lens flare effect does not
    /// perform any depth checking so if the directional light is occluded by an
    /// object in the scene, lens flare will pass through the object. The lens
    /// flare's color is set by the shaders and not exposed through an API in
    /// this class.
    class GZ_RENDERING_VISIBLE LensFlare
    {
      /// \brief Constructor
      public: LensFlare();

      /// \brief Destructor
      public: virtual ~LensFlare();

      /// \brief Set the camera which lensFlare will be applied to.
      /// \param[in] _camera Camera to be distorted
      public: void SetCamera(CameraPtr _camera);

      /// \brief Set the name of light that generates lens flare.
      /// \param[in] _name Light that generates lens flare
      public: void SetLightName(std::string _name);

      /// \brief Set the scale of lens flare. Must be greater than 0.
      /// \param[in] _scale Scale of lens flare
      public: void SetScale(const double _scale);

      /// \brief Set the color of lens flare.
      /// \param[in] _color Color of lens flare
      public: void SetColor(const ignition::math::Vector3d &_color);

      /// \brief Set the number of steps to take in each direction when
      /// checking for occlusions.
      /// \param[in] _occlusionSteps number of steps to take in each direction
      /// when checking for occlusion.
      public: void SetOcclusionSteps(double _occlusionSteps);

      /// \brief Set the name of the lens flare compositor to use the next
      /// time SetCamera is called.
      /// \param[in] _name Name of the compositor to use
      public: void SetCompositorName(const std::string &_name);

      /// \brief Update function to search light source
      protected: void Update();

      /// \brief Set camera stored in LensFlarePrivate
      /// \param[in] _camera Camera to use in sensor.
      protected: void SetCameraImpl(CameraPtr _camera);

      /// \brief Set lensFlareCompositorListener stored in LensFlarePrivate
      /// \param[in] _listener Shared pointer to object to be set.
      protected: void SetLensFlareCompositorListener(
          std::shared_ptr<LensFlareCompositorListener> _listener);

      /// \brief Set lensFlareInstance stored in LensFlarePrivate
      /// \param[in] _instance CompositorInstance to set
      protected: void SetLensFlareInstance(Ogre::CompositorInstance *_instance);

      /// \brief Request callback
      /// \param[in] _msg The message data.
      private: void OnRequest(ConstRequestPtr &_msg);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<LensFlarePrivate> dataPtr;
    };
    /// \}
  }
}
#endif
