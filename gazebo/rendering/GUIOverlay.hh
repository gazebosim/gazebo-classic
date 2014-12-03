/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GUI_OVERLAY_HH_
#define _GUI_OVERLAY_HH_

#ifdef _WIN32
  // Oh, yeah, CreateWindow is taken, too.
  #include <windows.h>
  #undef CreateWindow
#endif

#include <string>
#include <map>

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/math/Vector2d.hh"

#include "gazebo/rendering/cegui.h"
#include "gazebo/rendering/RenderTypes.hh"

namespace Ogre
{
  class RenderTarget;
}

#ifdef HAVE_CEGUI
namespace CEGUI
{
  class OgreRenderer;
  class Window;
}
#endif

namespace gazebo
{
  namespace rendering
  {
    class GUIOverlayPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class GUIOverlay GUIOverlay.hh rendering/rendering.hh
    /// \brief A class that creates a CEGUI overlay on a render window
    class GUIOverlay
    {
      /// \brief Constructor
      public: GUIOverlay();

      /// \brief Destructor
      public: virtual ~GUIOverlay();

      /// \brief Initialize the overlay
      /// \param[in] _renderTarget The render target which will have the
      /// overlay.
      public: void Init(Ogre::RenderTarget *_renderTarget);

      /// \brief Make the overlay invisible
      public: void Hide();

      /// \brief Make the overlay visible
      public: void Show();

      /// \brief Update the overlay's objects
      public: void Update();

      /// \brief Return true if the overlay has been initialized
      /// \return True if initialized
      public: bool IsInitialized();

      /// \brief Create a new window on the overlay.
      /// \param[in] _type The window type. This should match a CEGUI window
      /// type. See CEGUI::WindowManager::getSingleton().createWindow().
      /// \param[in] _name Unique name for the window.
      /// \param[in] _parent Name of the parent window.
      /// \param[in] _position Position of the window within the parent.
      /// \param[in] _size Size of the window.
      /// \param[in] _text Display title of the window.
      public: void CreateWindow(const std::string &_type,
                                const std::string &_name,
                                const std::string &_parent,
                                const math::Vector2d &_position,
                                const math::Vector2d &_size,
                                const std::string &_text);

      /// \brief Handle a mouse event.
      /// \param[in] _evt The mouse event.
      /// \return True if the mouse event was handled.
      public: bool HandleMouseEvent(const common::MouseEvent &_evt);

      /// \brief Handle a key press event.
      /// \param[in] _key The key pressed.
      /// \return True if the key press event was handled.
      public: bool HandleKeyPressEvent(const std::string &_key);

      /// \brief Handle a key release event.
      /// \param[in] _key The key released.
      /// \return True if the key release event was handled.
      public: bool HandleKeyReleaseEvent(const std::string &_key);

      /// \brief Load a CEGUI layout file
      /// \param[in] _filename Name of the layout file.
      public: void LoadLayout(const std::string &_filename);

      /// \brief Use this function to draw the output from a rendering::Camera
      /// to and overlay window.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _windowName Name of the window to receive the camera
      /// image
      /// \return True if successful
      public: bool AttachCameraToImage(CameraPtr &_camera,
                                       const std::string &_windowName);

      /// \brief Use this function to draw the output from
      /// a rendering::DepthCamera
      /// to and overlay window.
      /// \param[in] _camera Pointer to the camera.
      /// \param[in] _windowName Name of the window to receive the camera
      /// image
      /// \return True if successful
      public: bool AttachCameraToImage(DepthCameraPtr &_camera,
                                       const std::string &_windowName);

      /// \brief Resize the window
      public: void Resize(unsigned int _width, unsigned int _height);

      /// \brief PreRender function callback
      private: void PreRender();

      /// \brief Get the ASCII keyboard code based on a unicode character.
      /// \param[in] _unicode Unicode character.
      /// \return ASCII integer
      private: int GetKeyCode(const std::string  &_unicode);

#ifdef HAVE_CEGUI
      /// \brief Get a CEGUI::Window pointer to a named window.
      /// \param[in] _name Name of the window to retrieve.
      /// \return Pointer to the window.
      public: CEGUI::Window *GetWindow(const std::string &_name);

      /// \brief Load a CEGUI layout file.
      /// \param[in] _filename Name of the layout file.
      /// \return Pointer to the newly created CEGUI window.
      private: CEGUI::Window *LoadLayoutImpl(const std::string &_filename);

      /// \brief Button click callback.
      /// \param[in] _e CEGUI button event.
      /// \return True if handled.
      private: bool OnButtonClicked(const CEGUI::EventArgs &_e);
#endif

      /// \brief Register a CEGUI button callback.
      ///
      /// Assign a callback to a name button.
      /// \param[in] _buttonName Name of the button.
      /// \param[in] _fp Function pointer to the callback.
      /// \param[in] _obj Class pointer that contains _fp.
      public: template<typename T>
              void ButtonCallback(const std::string &_buttonName,
                                   void (T::*_fp)(), T *_obj)
              {
#ifdef HAVE_CEGUI
                CEGUI::Window *buttonWindow;
                buttonWindow =
                  CEGUI::WindowManager::getSingletonPtr()->getWindow(
                      _buttonName);
                buttonWindow->subscribeEvent(CEGUI::PushButton::EventClicked,
                  CEGUI::Event::Subscriber(&GUIOverlay::OnButtonClicked, this));

                this->callbacks[_buttonName] = boost::bind(_fp, _obj);
#else
                if (_buttonName && _fp && _obj)
                {
                  gzerr << "CEGUI not installed." << std::endl;
                }
#endif
              }

      /// \brief Map of callback functions to names
      public: std::map<std::string, boost::function<void()> > callbacks;

      /// \internal
      /// \brief Pointer to private data.
      private: GUIOverlayPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
