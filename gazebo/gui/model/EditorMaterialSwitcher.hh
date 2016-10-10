/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_MODEL_EDITORMATERIALSWITCHER_HH_
#define GAZEBO_GUI_MODEL_EDITORMATERIALSWITCHER_HH_

#include <string>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    // forward declarations
    class EditorMaterialListener;
    class EditorRenderTargetListener;

    // typedefs that are used only in this class
    using EditorRenderTargetListenerPtr =
        std::shared_ptr<EditorRenderTargetListener>;
    using EditorMaterialListenerPtr =
        std::shared_ptr<EditorMaterialListener>;

    /// \class EditorMaterialSwitcher EditorMaterialSwitcher.hh
    /// \brief Material switcher for the model editor used to toggle the
    /// material of the model.
    class GZ_GUI_VISIBLE EditorMaterialSwitcher
    {
      /// \brief Constructor
      /// \param[in] _camera Pointer to the camera whose viewport will be
      /// updated to see the effect of the material switch.
      public: EditorMaterialSwitcher(const rendering::CameraPtr &_camera);

      /// \brief Destructor
      public: ~EditorMaterialSwitcher() = default;

      /// \brief Set the material scheme that will be applied to the models
      /// in the editor
      /// \param[in] _scheme Name of material scheme
      public: void SetMaterialScheme(const std::string &_scheme);

      /// \brief Get the material scheme applied to the models in the editor
      /// \return Name of material scheme
      public: std::string MaterialScheme() const;

      /// \brief Ogre render target listener that adds and removes the
      /// material listener on every render event
      private: EditorRenderTargetListenerPtr renderTargetListener;

      /// \brief Ogre material listener that will handle switching the
      /// material scheme
      private: EditorMaterialListenerPtr materialListener;

      /// \brief Pointer to the camera
      private: rendering::CameraPtr camera;

      /// \brief Name of the original material scheme
      private: std::string originalMaterialScheme;

      /// \brief Name of the material scheme being used.
      private: std::string materialScheme;
    };

    /// \class EditorRenderTargetListener EditorMaterialSwitcher.hh
    /// \brief Ogre render target listener.
    class EditorRenderTargetListener : public Ogre::RenderTargetListener
    {
      /// \brief Constructor
      /// \param[in] _switcher Material listener that will be added to or
      /// removed from Ogre material manager's list of listeners.
      public: EditorRenderTargetListener(
                  const EditorMaterialListenerPtr &_switcher);

      /// \brief Destructor
      public: ~EditorRenderTargetListener() = default;

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

      /// \brief Pointer to the material listener
      private: EditorMaterialListenerPtr materialListener;
    };

    /// \class EditorMaterialListener EditorMaterialSwitcher.hh
    /// \brief Ogre material listener.
    class EditorMaterialListener : public Ogre::MaterialManager::Listener
    {
      /// \brief Constructor
      /// \param[in] _camera Pointer to the camera whose viewport will be
      /// updated to see the effect of the material switch.
      public: EditorMaterialListener(const rendering::CameraPtr &_camera);

      /// \brief Destructor
      public: ~EditorMaterialListener() = default;

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

      /// \brief Pointer to the camera
      private: rendering::CameraPtr camera;
    };
  }
}
#endif
