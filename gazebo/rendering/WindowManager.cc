/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "gazebo/rendering/skyx/include/SkyX.h"

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
  RTShaderSystem::AttachViewport(_camera->GetViewport(), _camera->GetScene());
  if (_camera->GetScene()->skyx != NULL)
    this->windows[_windowId]->addListener(_camera->GetScene()->skyx);
}

//////////////////////////////////////////////////
int WindowManager::CreateWindow(const std::string &ogreHandle,
                                unsigned int width,
                                unsigned int height)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  params["parentWindowHandle"] = ogreHandle;
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
          stream.str(), width, height, false, &params);
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

void WindowManager::GetAttribute(unsigned int id,
    const std::string &attr, void *data)
{
  if (id >= this->windows.size())
    gzerr << "Invalid window id[" << id << "]\n";
  else
    this->windows[id]->getCustomAttribute(attr, data);
}

void WindowManager::Resize(unsigned int id, int width, int height)
{
  if (id >= this->windows.size())
  {
    gzerr << "Invalid window id[" << id << "]\n";
  }
  else
  {
    this->windows[id]->resize(width, height);
    this->windows[id]->windowMovedOrResized();
  }
}

void WindowManager::Moved(unsigned int id)
{
  if (id >= this->windows.size())
  {
    gzerr << "Invalid window id[" << id << "]\n";
  }
  else
  {
    this->windows[id]->windowMovedOrResized();
  }
}

//////////////////////////////////////////////////
float WindowManager::GetAvgFPS(unsigned int windowId)
{
  float avgFPS = 0;

  if (windowId < this->windows.size())
  {
    float lastFPS, bestFPS, worstFPS = 0;
    this->windows[windowId]->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);
  }

  return avgFPS;
}

//////////////////////////////////////////////////
unsigned int WindowManager::GetTriangleCount(unsigned int windowId)
{
  if (windowId < this->windows.size())
    return this->windows[windowId]->getTriangleCount();
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
