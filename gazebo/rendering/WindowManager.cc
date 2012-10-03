/*
 * Copyright 2011 Nate Koenig
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

#include "rendering/ogre_gazebo.h"

#include "common/Events.hh"
#include "common/Color.hh"
#include "common/Console.hh"
#include "common/Exception.hh"

#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/WindowManager.hh"

using namespace gazebo;
using namespace rendering;


unsigned int WindowManager::windowCounter = 0;

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
                                unsigned int _width,
                                unsigned int _height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  params["parentWindowHandle"] = _ogreHandle;
  params["externalGLControl"] = true;
  params["FSAA"] = "4";

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
void WindowManager::GetAttribute(unsigned int _id,
    const std::string &_attr, void *_data)
{
  if (_id >= this->windows.size())
    gzerr << "Invalid window id[" << _id << "]\n";
  else
    this->windows[_id]->getCustomAttribute(_attr, _data);
}

/////////////////////////////////////////////////
void WindowManager::Resize(unsigned int _id, int _width, int _height)
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
void WindowManager::Moved(unsigned int _id)
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
float WindowManager::GetAvgFPS(unsigned int _windowId)
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
unsigned int WindowManager::GetTriangleCount(unsigned int _windowId)
{
  if (_windowId < this->windows.size())
    return this->windows[_windowId]->getTriangleCount();
  else
    return 0;
}

//////////////////////////////////////////////////
Ogre::RenderWindow *WindowManager::GetWindow(unsigned int _id)
{
  if (_id < this->windows.size())
    return this->windows[_id];
  else
    return NULL;
}
