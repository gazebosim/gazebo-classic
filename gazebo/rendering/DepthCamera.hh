/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef _GAZEBO_RENDERING_DEPTHCAMERA_HH_
#define _GAZEBO_RENDERING_DEPTHCAMERA_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Material;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace rendering
  {
    // Forward declare private data.
    class DepthCameraPrivate;
    class ReflectanceRenderTargetListener;
    class ReflectanceMaterialListener;
    class ReflectanceMaterialSwitcher;

    // typedefs that are used only in this class
    using ReflectanceRenderTargetListenerPtr =
        std::shared_ptr<ReflectanceRenderTargetListener>;
    using ReflectanceMaterialListenerPtr =
        std::shared_ptr<ReflectanceMaterialListener>;
    using ReflectanceMaterialSwitcherPtr =
        std::shared_ptr<ReflectanceMaterialSwitcher>;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class DepthCamera DepthCamera.hh rendering/rendering.hh
    /// \brief Depth camera used to render depth data into an image buffer
    class GZ_RENDERING_VISIBLE DepthCamera : public Camera
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      public: DepthCamera(const std::string &_namePrefix,
                          ScenePtr _scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~DepthCamera();

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: void Load(sdf::ElementPtr _sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// Finalize the camera
      public: void Fini();

      /// \brief Create a texture which will hold the depth data
      /// \param[in] _textureName Name of the texture to create
      public: void CreateDepthTexture(const std::string &_textureName);

      /// \brief Create a texture which will hold the reflectance data
      /// \param[in] _textureName Name of the texture to create
      public: void CreateReflectanceTexture(const std::string &_textureName);

      /// \brief Render the camera
      public: virtual void PostRender();

      /// \brief All things needed to get back z buffer for depth data
      /// \return The z-buffer as a float array
      public: virtual const float *DepthData() const;

      /// \brief Set the render target, which renders the depth data
      /// \param[in] _target Pointer to the render target
      public: virtual void SetDepthTarget(Ogre::RenderTarget *_target);

      /// \brief Connect a to the new depth image signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: event::ConnectionPtr ConnectNewDepthFrame(
          std::function<void (const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);

      /// \brief Connect a to the new rgb point cloud signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: event::ConnectionPtr ConnectNewRGBPointCloud(
          std::function<void (const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);

      /// \brief Connect a to the new reflectance data
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: event::ConnectionPtr ConnectNewReflectanceFrame(
          std::function<void (const float *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber);

      /// \brief Implementation of the render call
      private: virtual void RenderImpl();

      /// \brief Update a render target
      /// \param[in] _target Render target to update
      /// \param[in] _material Material to use
      /// \param[in] _matName Material name
      private: void UpdateRenderTarget(Ogre::RenderTarget *_target,
                                       Ogre::Material *_material,
                                       const std::string &_matName);

      /// \brief Pointer to the depth texture
      protected: Ogre::Texture *depthTexture;

      /// \brief Pointer to the depth target
      protected: Ogre::RenderTarget *depthTarget = nullptr;

      /// \brief Pointer to the depth viewport
      protected: Ogre::Viewport *depthViewport;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<DepthCameraPrivate> dataPtr;
    };

    /// \class ReflectanceMaterialSwitcher ReflectanceMaterialSwitcher.hh
    /// \brief Material switcher for reflectance
    class GZ_RENDERING_VISIBLE ReflectanceMaterialSwitcher
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to get the visuals
      /// \param[in] viewport will be updated to see the effect of
      /// the material switch.
      public: explicit ReflectanceMaterialSwitcher(
                  ScenePtr _scene, Ogre::Viewport* _viewport);

      /// \brief Destructor
      public: ~ReflectanceMaterialSwitcher() = default;

      /// \brief Set the material scheme that will be applied to the models
      /// in the editor
      /// \param[in] _scheme Name of material scheme
      public: void SetMaterialScheme(const std::string &_scheme);

      /// \brief Get the material scheme applied to the models in the editor
      /// \return Name of material scheme
      public: std::string MaterialScheme() const;

      /// \brief Ogre render target listener that adds and removes the
      /// material listener on every render event
      private: ReflectanceRenderTargetListenerPtr renderTargetListener;

      /// \brief Ogre material listener that will handle switching the
      /// material scheme
      private: ReflectanceMaterialListenerPtr materialListener;

      /// \brief viewport pointer to reflectance
      private: Ogre::Viewport* viewport;

      /// \brief Name of the original material scheme
      private: std::string originalMaterialScheme;

      /// \brief Name of the material scheme being used.
      private: std::string materialScheme;
    };


    /// \class ReflectanceRenderTargetListener
    /// \brief Ogre render target listener.
    class ReflectanceRenderTargetListener : public Ogre::RenderTargetListener
    {
      /// \brief Constructor
      /// \param[in] _switcher Material listener that will be added to or
      /// removed from Ogre material manager's list of listeners.
      public: explicit ReflectanceRenderTargetListener(
                          const ReflectanceMaterialListenerPtr &_switcher);

      /// \brief Destructor
      public: ~ReflectanceRenderTargetListener() = default;

      /// \brief Ogre's pre-render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void preRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Ogre's post-render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void postRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Reflectance material listener pointer
      private: ReflectanceMaterialListenerPtr materialListener;
    };

    /// \class ReflectanceMaterialListener ReflectanceMaterialListener.hh
    /// \brief reflectance material listener.
    class ReflectanceMaterialListener : public Ogre::MaterialManager::Listener
    {
      /// \brief Constructor
      /// \param[in] _scene Pointer to get the visuals.
      public: explicit ReflectanceMaterialListener(ScenePtr _scene);

      /// \brief Destructor
      public: ~ReflectanceMaterialListener() = default;

      /// \brief Ogre callback that is used to specify the material to use when
      /// the requested scheme is not found
      /// \param[in] _schemeIndex Index of scheme requested
      /// \param[in] _schemeName Name of scheme requested
      /// \param[in] _originalMaterial Orignal material that does not contain
      /// the requested scheme
      /// \param[in] _lodIndex The material level-of-detail
      /// \param[in] _rend Pointer to the Ogre::Renderable object requesting
      /// the use of the techinique
      /// \return The Ogre material technique to use when scheme is not found.
      public: virtual Ogre::Technique *handleSchemeNotFound(
                  uint16_t _schemeIndex, const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial, uint16_t _lodIndex,
                  const Ogre::Renderable *_rend);

      /// \brief Scene pointer
      private: ScenePtr scene;
    };
    /// \}
  }
}
#endif
