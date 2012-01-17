#ifndef GUI_OVERLAY
#define GUI_OVERLAY

#include <string>
#include "gazebo_config.h"

#ifdef HAVE_CEGUI
#include "CEGUI/CEGUI.h"
#include "CEGUI/CEGUIEventArgs.h"
#endif

#include "common/MouseEvent.hh"
#include "common/Events.hh"

#include "math/MathTypes.hh"

#include "rendering/RenderTypes.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"

namespace Ogre
{
  class RenderTarget;
}

namespace CEGUI
{
  class OgreRenderer;
  class Window;
}

namespace gazebo
{
  namespace rendering
  {
    class GUIOverlay
    {
      public: GUIOverlay();
      public: virtual ~GUIOverlay();

      public: void Init(Ogre::RenderTarget *_renderTarget);

      public: void Hide();
      public: void Show();

      public: void Update();

      public: bool IsInitialized();

      public: void CreateWindow(const std::string &_type,
                                 const std::string &_name,
                                 const std::string &_parent,
                                 const math::Vector2d &_position,
                                 const math::Vector2d &_size,
                                 const std::string &_text);

      public: bool HandleMouseEvent(const common::MouseEvent &_evt);
      public: bool HandleKeyPressEvent(const std::string &_key);
      public: bool HandleKeyReleaseEvent(const std::string &_key);

      /// \brief Load a CEGUI layout file
      public: void LoadLayout(const std::string &_filename);

      public: bool AttachCameraToImage(CameraPtr &_camera,
                                       const std::string &_windowName);

      public: bool AttachCameraToImage(DepthCameraPtr &_camera,
                                       const std::string &_windowName);


      public: void Resize(unsigned int _width, unsigned int _height);
      private: void PreRender();


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
#endif
              }

#ifdef HAVE_CEGUI
      public: CEGUI::Window *GetWindow(const std::string &_name);

      /// Load a CEGUI layout file
      private: CEGUI::Window *LoadLayoutImpl(const std::string &_filename);

      private: bool OnButtonClicked(const CEGUI::EventArgs& _e);
      private: int GetKeyCode(const std::string  &_unicode);

      private: CEGUI::OgreRenderer *guiRenderer;
#endif

      private: std::vector<event::ConnectionPtr> connections;
      private: std::string layoutFilename;
      private: std::map<std::string, boost::function<void()> > callbacks;

      private: unsigned int rttImageSetCount;
      private: bool initialized;
    };
  }
}

#endif

