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

#include "gazebo/rendering/GUIOverlay.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/common/SystemPaths.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/math/Vector2d.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/DepthCamera.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
GUIOverlay::GUIOverlay()
{
  this->initialized = false;
#ifdef HAVE_CEGUI
  this->guiRenderer = NULL;

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&GUIOverlay::PreRender, this)));
#endif
  this->rttImageSetCount = 0;
}

/////////////////////////////////////////////////
GUIOverlay::~GUIOverlay()
{
  this->initialized = false;
#ifdef HAVE_CEGUI
  CEGUI::OgreRenderer::destroySystem();
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
void GUIOverlay::Init(Ogre::RenderTarget *_renderTarget)
{
  if (this->initialized)
    return;

  CEGUI::System *system = CEGUI::System::getSingletonPtr();

  if (system)
    return;

  std::string logPath = common::SystemPaths::Instance()->GetLogPath();
  logPath += "/cegui.log";

  this->guiRenderer = &CEGUI::OgreRenderer::create(*_renderTarget);
  CEGUI::OgreResourceProvider &ip =
    CEGUI::OgreRenderer::createOgreResourceProvider();
  CEGUI::OgreImageCodec &ic = CEGUI::OgreRenderer::createOgreImageCodec();

  CEGUI::System::create(*((CEGUI::Renderer*)this->guiRenderer),
      (CEGUI::ResourceProvider*)(&ip),
      static_cast<CEGUI::XMLParser*>(0),
      (CEGUI::ImageCodec*)(&ic),
      NULL, "", logPath);

  CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
  CEGUI::Font::setDefaultResourceGroup("Fonts");
  CEGUI::Scheme::setDefaultResourceGroup("Schemes");
  CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
  CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
  CEGUI::AnimationManager::setDefaultResourceGroup("Animations");

  CEGUI::SchemeManager::getSingleton().create("TaharezLook.scheme");
  CEGUI::SchemeManager::getSingleton().create("VanillaSkin.scheme");
  CEGUI::SchemeManager::getSingleton().create("GazeboSkin.scheme");
  CEGUI::FontManager::getSingleton().create("DejaVuSans-10.font");

  // clearing this queue actually make sure it's created
  this->guiRenderer->getDefaultRenderingRoot().clearGeometry(CEGUI::RQ_OVERLAY);

  // Create a root window, and set it as the root window
  CEGUI::Window *rootWindow =
    CEGUI::WindowManager::getSingleton().createWindow("DefaultWindow", "root");
  rootWindow->setMousePassThroughEnabled(true);
  CEGUI::System::getSingleton().setGUISheet(rootWindow);
  this->initialized = true;
}
#else
void GUIOverlay::Init(Ogre::RenderTarget * /*_renderTarget*/)
{
  this->initialized = true;
}
#endif

/////////////////////////////////////////////////
void GUIOverlay::Hide()
{
#ifdef HAVE_CEGUI
  CEGUI::System::getSingletonPtr()->getGUISheet()->hide();
#endif
}

/////////////////////////////////////////////////
void GUIOverlay::Show()
{
#ifdef HAVE_CEGUI
  CEGUI::System::getSingletonPtr()->getGUISheet()->show();
#endif
}

/////////////////////////////////////////////////
void GUIOverlay::Update()
{
#ifdef HAVE_CEGUI
  CEGUI::System::getSingleton().injectTimePulse(0.01);
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
void GUIOverlay::CreateWindow(const std::string &_type,
                               const std::string &_name,
                               const std::string &_parent,
                               const math::Vector2d &_position,
                               const math::Vector2d &_size,
                               const std::string &_text)
{
  CEGUI::Window *parent =
    CEGUI::WindowManager::getSingleton().getWindow(_parent);
  CEGUI::Window *window =
    CEGUI::WindowManager::getSingleton().createWindow(_type, _name);

  parent->addChildWindow(window);

  window->setPosition(CEGUI::UVector2(CEGUI::UDim(_position.x, 0),
                                             CEGUI::UDim(_position.y, 0)));

  window->setSize(CEGUI::UVector2(CEGUI::UDim(_size.x, 0),
                                         CEGUI::UDim(_size.y, 0)));
  window->setText(_text);
}
#else
void GUIOverlay::CreateWindow(const std::string &/*_type*/,
                               const std::string &/*_name*/,
                               const std::string &/*_parent*/,
                               const math::Vector2d &/*_position*/,
                               const math::Vector2d &/*_size*/,
                               const std::string &/*_text*/)
{
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
bool GUIOverlay::HandleKeyPressEvent(const std::string &_key)
{
  CEGUI::System *system = CEGUI::System::getSingletonPtr();
  int unicode = static_cast<int>(_key[0]);
  if (unicode >= 32 && unicode <= 126)
    system->injectChar(unicode);
  else
    system->injectKeyDown(this->GetKeyCode(_key));
  return true;
}
#else
bool GUIOverlay::HandleKeyPressEvent(const std::string &/*_key*/)
{
  return true;
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
bool GUIOverlay::HandleKeyReleaseEvent(const std::string &_key)
{
  CEGUI::System *system = CEGUI::System::getSingletonPtr();
  int unicode = static_cast<int>(_key[0]);
  if (unicode <= 32 || unicode >= 126)
    system->injectKeyUp(this->GetKeyCode(_key));
  return true;
}
#else
bool GUIOverlay::HandleKeyReleaseEvent(const std::string &/*_key*/)
{
  return true;
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
bool GUIOverlay::HandleMouseEvent(const common::MouseEvent &_evt)
{
  bool result = false;
  bool press, release, pos, scroll;

  press = false;
  release = false;
  scroll = false;

  CEGUI::System *system = CEGUI::System::getSingletonPtr();
  pos = system->injectMousePosition(_evt.pos.x, _evt.pos.y);

  if (_evt.type == common::MouseEvent::PRESS)
  {
    if (_evt.button == common::MouseEvent::LEFT)
      press = system->injectMouseButtonDown(CEGUI::LeftButton);
    if (_evt.button == common::MouseEvent::RIGHT)
      press = system->injectMouseButtonDown(CEGUI::RightButton);
    if (_evt.button == common::MouseEvent::MIDDLE)
      press = system->injectMouseButtonDown(CEGUI::MiddleButton);
  }

  if (_evt.type == common::MouseEvent::RELEASE)
  {
    if (_evt.button == common::MouseEvent::LEFT)
      release = system->injectMouseButtonUp(CEGUI::LeftButton);
    if (_evt.button == common::MouseEvent::RIGHT)
      release = system->injectMouseButtonUp(CEGUI::RightButton);
    if (_evt.button == common::MouseEvent::MIDDLE)
      release = system->injectMouseButtonUp(CEGUI::MiddleButton);
  }

  if (_evt.type == common::MouseEvent::SCROLL)
    scroll = system->injectMouseWheelChange(-1 * _evt.scroll.y);

  result = pos || release || press || scroll;

  return result;
}
#else
bool GUIOverlay::HandleMouseEvent(const common::MouseEvent &/*_evt*/)
{
  return false;
}
#endif

/////////////////////////////////////////////////
bool GUIOverlay::IsInitialized()
{
#ifdef HAVE_CEGUI
  return CEGUI::WindowManager::getSingletonPtr() != NULL;
#else
  return false;
#endif
}

/////////////////////////////////////////////////
void GUIOverlay::LoadLayout(const std::string &_filename)
{
  this->layoutFilename = _filename;
}

/////////////////////////////////////////////////
void GUIOverlay::PreRender()
{
#ifdef HAVE_CEGUI
  if (this->IsInitialized() && !this->layoutFilename.empty())
  {
    this->LoadLayoutImpl(this->layoutFilename);
    this->layoutFilename.clear();
  }
#endif
}

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
CEGUI::Window *GUIOverlay::LoadLayoutImpl(const std::string &_filename)
{
  CEGUI::Window *window = NULL;
  CEGUI::Window *rootWindow = NULL;

  CEGUI::WindowManager *windowManager = CEGUI::WindowManager::getSingletonPtr();
  if (!windowManager)
  {
    gzerr << "Attempting to create a GUI overlay window before load\n";
    return window;
  }

  rootWindow = windowManager->getWindow("root");
  if (rootWindow)
  {
    window = windowManager->loadWindowLayout(_filename);
    if (window->getType() == "DefaultWindow")
      window->setMousePassThroughEnabled(true);
    rootWindow->addChildWindow(window);
  }
  else
  {
    gzerr << "Attempting to create a GUI overlay window before load\n";
  }

  return window;
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
void GUIOverlay::Resize(unsigned int _width, unsigned int _height)
{
  if (this->guiRenderer)
  {
    this->guiRenderer->setDisplaySize(CEGUI::Size(_width, _height));

    CEGUI::WindowManager *windowManager =
      CEGUI::WindowManager::getSingletonPtr();

    if (windowManager && windowManager->isWindowPresent("root"))
    {
      CEGUI::Window *rootWindow = windowManager->getWindow("root");
      rootWindow->setArea(CEGUI::UDim(0, 0), CEGUI::UDim(0, 0),
          CEGUI::UDim(1, 0), CEGUI::UDim(1, 0));
    }
  }
}
#else
void GUIOverlay::Resize(unsigned int /*_width*/, unsigned int /*_height*/)
{
}
#endif

/////////////////////////////////////////////////
bool GUIOverlay::AttachCameraToImage(DepthCameraPtr &_camera,
                                     const std::string &_windowName)
{
  CameraPtr cam = boost::dynamic_pointer_cast<Camera>(_camera);
  return this->AttachCameraToImage(cam , _windowName);
}

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
bool GUIOverlay::AttachCameraToImage(CameraPtr &_camera,
                                     const std::string &_windowName)
{
  bool result = false;
  CEGUI::Window *window = NULL;
  CEGUI::WindowManager *windowManager = CEGUI::WindowManager::getSingletonPtr();

  if (!windowManager)
  {
    gzerr << "CEGUI system not initialized\n";
    return false;
  }

  window = windowManager->getWindow(_windowName);
  if (!window)
  {
    gzerr << "Unable to find window[" << _windowName << "]\n";
    return false;
  }

  if (!_camera->GetRenderTexture())
  {
    gzerr << "Camera does not have a render texture\n";
    return false;
  }

  Ogre::TexturePtr texPtr(_camera->GetRenderTexture());
  CEGUI::Texture &guiTex = this->guiRenderer->createTexture(
      texPtr);

  this->rttImageSetCount++;
  std::ostringstream stream;
  stream << "RTTImageset_" << this->rttImageSetCount;

  CEGUI::Imageset &imageSet = CEGUI::ImagesetManager::getSingleton().create(
      stream.str().c_str(), guiTex);

  imageSet.defineImage("RTTImage", CEGUI::Point(0.0f, 0.0f),
      CEGUI::Size(guiTex.getSize().d_width,
                  guiTex.getSize().d_height),
      CEGUI::Point(0.0f, 0.0f));

  window->setProperty("Image",
      CEGUI::PropertyHelper::imageToString(&imageSet.getImage("RTTImage")));
  result = true;
  return result;
}
#else
bool GUIOverlay::AttachCameraToImage(CameraPtr &/*_camera*/,
                                     const std::string &/*_windowName*/)
{
  return true;
}
#endif

/////////////////////////////////////////////////
#ifdef HAVE_CEGUI
bool GUIOverlay::OnButtonClicked(const CEGUI::EventArgs& _e)
{
  std::map<std::string, boost::function<void()> >::iterator iter;
  CEGUI::WindowEventArgs *args = (CEGUI::WindowEventArgs*)(&_e);
  std::string name = args->window->getName().c_str();

  iter = this->callbacks.find(name);
  if (iter != this->callbacks.end())
    (iter->second) ();

  return true;
}

/////////////////////////////////////////////////
CEGUI::Window *GUIOverlay::GetWindow(const std::string &_name)
{
  return CEGUI::WindowManager::getSingletonPtr()->getWindow(_name);
}

/////////////////////////////////////////////////
int GUIOverlay::GetKeyCode(const std::string  &_unicode)
{
  switch (_unicode[0])
  {
    case 'A':
    case 'a': return CEGUI::Key::A;
    case 'B':
    case 'b': return CEGUI::Key::B;
    case 'C':
    case 'c': return CEGUI::Key::C;
    case 'D':
    case 'd': return CEGUI::Key::D;
    case 'E':
    case 'e': return CEGUI::Key::E;
    case 'F':
    case 'f': return CEGUI::Key::F;
    case 'G':
    case 'g': return CEGUI::Key::G;
    case 'H':
    case 'h': return CEGUI::Key::H;
    case 'I':
    case 'i': return CEGUI::Key::I;
    case 'J':
    case 'j': return CEGUI::Key::J;
    case 'K':
    case 'k': return CEGUI::Key::K;
    case 'L':
    case 'l': return CEGUI::Key::L;
    case 'M':
    case 'm': return CEGUI::Key::M;
    case 'N':
    case 'n': return CEGUI::Key::N;
    case 'O':
    case 'o': return CEGUI::Key::O;
    case 'P':
    case 'p': return CEGUI::Key::P;
    case 'Q':
    case 'q': return CEGUI::Key::Q;
    case 'R':
    case 'r': return CEGUI::Key::R;
    case 'S':
    case 's': return CEGUI::Key::S;
    case 'T':
    case 't': return CEGUI::Key::T;
    case 'U':
    case 'u': return CEGUI::Key::U;
    case 'V':
    case 'v': return CEGUI::Key::V;
    case 'W':
    case 'w': return CEGUI::Key::W;
    case 'X':
    case 'x': return CEGUI::Key::X;
    case 'Y':
    case 'y': return CEGUI::Key::Y;
    case 'Z':
    case 'z': return CEGUI::Key::Z;
    case '1': return CEGUI::Key::One;
    case '2': return CEGUI::Key::Two;
    case '3': return CEGUI::Key::Three;
    case '4': return CEGUI::Key::Four;
    case '5': return CEGUI::Key::Five;
    case '6': return CEGUI::Key::Six;
    case '7': return CEGUI::Key::Seven;
    case '8': return CEGUI::Key::Eight;
    case '9': return CEGUI::Key::Nine;
    case '0': return CEGUI::Key::Zero;
    case '-': return CEGUI::Key::Minus;
    case '=': return CEGUI::Key::Equals;
    case '[': return CEGUI::Key::LeftBracket;
    case ']': return CEGUI::Key::RightBracket;
    case ';': return CEGUI::Key::Semicolon;
    case '\'': return CEGUI::Key::Apostrophe;
    case '`': return CEGUI::Key::Grave;
    case '\\': return CEGUI::Key::Backslash;
    case ',': return CEGUI::Key::Comma;
    case '.': return CEGUI::Key::Period;
    case '/': return CEGUI::Key::Slash;
    case ':': return CEGUI::Key::Colon;
    case ' ': return CEGUI::Key::Space;
    case '_': return CEGUI::Key::Underline;
    case '*': return CEGUI::Key::Multiply;
    default:
              break;
  };

  switch (static_cast<int>(_unicode[0]))
  {
    case 8: return CEGUI::Key::Backspace;
    case 27: return CEGUI::Key::Escape;
    case 13: return CEGUI::Key::Return;
    case 127: return CEGUI::Key::Delete;
    default:
              break;
  };

  return 0;
}
#endif
