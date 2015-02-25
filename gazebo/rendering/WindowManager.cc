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
#include <math.h>

#ifdef  __APPLE__
# include <QtCore/qglobal.h>
#endif

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/WindowManager.hh"

using namespace gazebo;
using namespace rendering;


uint32_t WindowManager::windowCounter = 0;

//////////////////////////////////////////////////
WindowManager::WindowManager()
{
}

//////////////////////////////////////////////////
WindowManager::~WindowManager()
{
  this->Fini();
}

//////////////////////////////////////////////////
void WindowManager::Fini()
{
  // TODO: this was causing a segfault on shutdown
  /*for (std::vector<Ogre::RenderWindow*>::iterator iter = this->windows.begin();
      iter != this->windows.end(); iter++)
  {
    (*iter)->removeAllViewports();
    (*iter)->destroy();
  }*/
  this->windows.clear();
}

//////////////////////////////////////////////////
void WindowManager::SetCamera(int _windowId, CameraPtr _camera)
{
  this->windows[_windowId]->removeAllViewports();
  _camera->SetRenderTarget(this->windows[_windowId]);
}

//////////////////////////////////////////////////
int WindowManager::CreateWindow(const std::string &_ogreHandle,
                                uint32_t _width,
                                uint32_t _height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

#ifdef Q_OS_MAC
  params["externalWindowHandle"] = _ogreHandle;
#else
  params["parentWindowHandle"] = _ogreHandle;
#endif
  params["externalGLControl"] = true;

  if (_width > 1 && _height > 1)
  {
    params["FSAA"] = "4";
    params["stereoMode"] = "Frame Sequential";
  }

  // Set the macAPI for Ogre based on the Qt implementation
#ifdef QT_MAC_USE_COCOA
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#else
  params["macAPI"] = "carbon";
#endif

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = RenderEngine::Instance()->root->createRenderWindow(
          stream.str(), _width, _height, false, &params);
    }
    catch(...)
    {
      gzerr << " Unable to create the rendering window\n";
      window = NULL;
    }
  }

  if (attempts >= 10)
  {
    gzthrow("Unable to create the rendering window\n");
  }

  if (window)
  {
    window->setActive(true);
    // window->setVisible(true);
    window->setAutoUpdated(false);

    this->windows.push_back(window);
  }

  return this->windows.size()-1;
}

/////////////////////////////////////////////////
void WindowManager::Resize(uint32_t _id, int _width, int _height)
{
  if (_id >= this->windows.size())
  {
    gzerr << "Invalid window id[" << _id << "]\n";
  }
  else
  {
    this->windows[_id]->resize(_width, _height);
    this->windows[_id]->windowMovedOrResized();
  }
}

/////////////////////////////////////////////////
void WindowManager::Moved(uint32_t _id)
{
  if (_id >= this->windows.size())
  {
    gzerr << "Invalid window id[" << _id << "]\n";
  }
  else
  {
    this->windows[_id]->windowMovedOrResized();
  }
}

//////////////////////////////////////////////////
float WindowManager::GetAvgFPS(uint32_t _windowId)
{
  float avgFPS = 0;

  if (_windowId < this->windows.size())
  {
    float lastFPS, bestFPS, worstFPS = 0;
    this->windows[_windowId]->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);
  }

  return avgFPS;
}

//////////////////////////////////////////////////
uint32_t WindowManager::GetTriangleCount(uint32_t _windowId)
{
  if (_windowId < this->windows.size())
    return this->windows[_windowId]->getTriangleCount();
  else
    return 0;
}

//////////////////////////////////////////////////
Ogre::RenderWindow *WindowManager::GetWindow(uint32_t _id)
{
  if (_id < this->windows.size())
    return this->windows[_id];
  else
    return NULL;
}
