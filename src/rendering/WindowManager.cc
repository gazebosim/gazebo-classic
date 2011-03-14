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
#include <Ogre.h>
#include <math.h>

#include "common/Events.hh"
#include "common/Color.hh"
#include "common/GazeboMessage.hh"
#include "common/GazeboError.hh"

#include "rendering/RenderEngine.hh"
//#include "rendering/RTShaderSystem.hh"
#include "rendering/Camera.hh"
#include "rendering/Conversions.hh"
#include "rendering/WindowManager.hh"

using namespace gazebo;
using namespace rendering;


unsigned int WindowManager::windowCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
WindowManager::WindowManager()
{
  this->renderConnection = event::Events::ConnectRenderSignal( boost::bind(&WindowManager::Render, this) );
}

////////////////////////////////////////////////////////////////////////////////
WindowManager::~WindowManager()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Attach a camera to a window
void WindowManager::SetCamera( int windowId, Camera *camera)
{
  Ogre::Viewport *viewport = NULL;

  this->windows[windowId]->removeAllViewports(); 
  viewport = this->windows[windowId]->addViewport(camera->GetCamera());

  double ratio = (double)viewport->getActualWidth() / (double)viewport->getActualHeight();
  double vfov = fabs(2.0 * atan(tan(camera->GetHFOV().GetAsRadian() / 2.0) / ratio));

  camera->SetAspectRatio( ratio );
  camera->GetCamera()->setFOVy(Ogre::Radian(vfov));

  viewport->setClearEveryFrame(true);
  viewport->setBackgroundColour( Conversions::Color(common::Color(0,0,0)) );
  viewport->setVisibilityMask(camera->GetVisibilityMask());

  //RTShaderSystem::AttachViewport(viewport, camera->GetScene());
}

////////////////////////////////////////////////////////////////////////////////
// Create a window
int WindowManager::CreateWindow( const std::string ogreHandle, 
                                                 unsigned int width, 
                                                 unsigned int height) 
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  params["parentWindowHandle"] = ogreHandle;

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";

  int attempts = 0;
  while (window == NULL && (attempts++) < 10)
  {
    try
    {
      window = RenderEngine::Instance()->root->createRenderWindow( stream.str(), 
                     width, height, false, &params);
    }
    catch (...)
    {
      gzerr(0) << " Unable to create the rendering window\n";
      window = NULL;
    }
  }

  if (attempts >= 10)
  {
    gzthrow("Unable to create the rendering window\n");
  }

  window->setActive(true);
  //window->setVisible(true);
  window->setAutoUpdated(true);

  this->windows.push_back(window);

  return this->windows.size()-1;
}

void WindowManager::Resize(unsigned int id, int width, int height)
{
  if (id >= this->windows.size())
  {
    gzerr(5) << "Invalid window id[" << id << "]\n";
  }
  else
  {
    this->windows[id]->resize(width, height);
    this->windows[id]->windowMovedOrResized();
  }
} 

void WindowManager::Render()
{
  for (unsigned int i=0; i < this->windows.size(); i++)
  {
    this->windows[i]->update(false);
    this->windows[i]->swapBuffers();
  }
}
