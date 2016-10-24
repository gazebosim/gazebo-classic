/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo/rendering/WindowManagerPrivate.hh"

using namespace gazebo;
using namespace rendering;

uint32_t WindowManagerPrivate::windowCounter = 0;

//////////////////////////////////////////////////
WindowManager::WindowManager()
  : dataPtr(new WindowManagerPrivate)
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
  /*for (std::vector<Ogre::RenderWindow*>::iterator iter =
      this->dataPtr->windows.begin();
      iter != this->dataPtr->windows.end(); iter++)
  {
    (*iter)->removeAllViewports();
    (*iter)->destroy();
  }*/
  this->dataPtr->windows.clear();
}

//////////////////////////////////////////////////
void WindowManager::SetCamera(int _windowId, CameraPtr _camera)
{
  if (static_cast<unsigned int>(_windowId) < this->dataPtr->windows.size() &&
      this->dataPtr->windows[_windowId])
    this->dataPtr->windows[_windowId]->removeAllViewports();
  _camera->SetRenderTarget(this->dataPtr->windows[_windowId]);
}

//////////////////////////////////////////////////
int WindowManager::CreateWindow(const std::string &_ogreHandle,
                                uint32_t _width,
                                uint32_t _height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  // Mac and Windows *must* use externalWindow handle.
#if defined(__APPLE__) || defined(_MSC_VER)
  params["externalWindowHandle"] = _ogreHandle;
#else
  params["parentWindowHandle"] = _ogreHandle;
#endif
  params["FSAA"] = "4";
  params["stereoMode"] = "Frame Sequential";

  // Set the macAPI for Ogre
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";

  // Hide window if dimensions are less than or equal to one.
  if (_width <= 1 && _height <=1)
    params["border"] = "none";

  std::ostringstream stream;
  stream << "OgreWindow(" << this->dataPtr->windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = RenderEngine::Instance()->Root()->createRenderWindow(
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
    window->setVisible(true);
    window->setAutoUpdated(false);

    // Windows needs to reposition the render window to 0,0.
    window->reposition(0, 0);

    this->dataPtr->windows.push_back(window);
  }

  return this->dataPtr->windows.size()-1;
}

/////////////////////////////////////////////////
void WindowManager::Resize(uint32_t _id, int _width, int _height)
{
  if (_id >= this->dataPtr->windows.size())
  {
    gzerr << "Invalid window id[" << _id << "]\n";
  }
  else
  {
    this->dataPtr->windows[_id]->resize(_width, _height);
    this->dataPtr->windows[_id]->windowMovedOrResized();
  }
}

/////////////////////////////////////////////////
void WindowManager::Moved(uint32_t _id)
{
  if (_id >= this->dataPtr->windows.size())
  {
    gzerr << "Invalid window id[" << _id << "]\n";
  }
  else
  {
    this->dataPtr->windows[_id]->windowMovedOrResized();
  }
}

//////////////////////////////////////////////////
float WindowManager::AvgFPS(const uint32_t _windowId) const
{
  float avgFPS = 0;

  if (_windowId < this->dataPtr->windows.size())
  {
    float lastFPS, bestFPS, worstFPS = 0;
    this->dataPtr->windows[_windowId]->getStatistics(
        lastFPS, avgFPS, bestFPS, worstFPS);
  }

  return avgFPS;
}

//////////////////////////////////////////////////
uint32_t WindowManager::TriangleCount(const uint32_t _windowId) const
{
  if (_windowId < this->dataPtr->windows.size())
    return this->dataPtr->windows[_windowId]->getTriangleCount();
  else
    return 0;
}

//////////////////////////////////////////////////
Ogre::RenderWindow *WindowManager::Window(const uint32_t _id) const
{
  if (_id < this->dataPtr->windows.size())
    return this->dataPtr->windows[_id];
  else
    return NULL;
}
