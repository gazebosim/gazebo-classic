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
#ifndef WINDOWMANAGER_HH
#define WINDOWMANAGER_HH

#include "common/SingletonT.hh"
#include "rendering/RenderTypes.hh"

#include <string>
#include <vector>

namespace Ogre
{
  class RenderWindow;
}

namespace gazebo
{
	namespace rendering
  {
    class WindowManager : public SingletonT<WindowManager>
    {
      public: WindowManager();
      public: virtual ~WindowManager();
  
      public: int CreateWindow( std::string ogreHandle, 
                                unsigned int width, 
                                unsigned int height );
  
      public: void GetAttribute(unsigned int id, 
                  const std::string &attr, void *data);
  
      /// \brief Attach a camera to a window
      public: void SetCamera( int windowId, CameraPtr camera);
  
      /// \brief Resize a window
      public: void Resize(unsigned int id, int width, int height);
  
      public: void Moved(unsigned int id);

      /// \brief Get the average FPS
      public: float GetAvgFPS(unsigned int windowId);
  
      /// \brief Get the triangle count
      public: unsigned int GetTriangleCount(unsigned int windotId);
 
      private: std::vector<Ogre::RenderWindow *> windows;
  
      private: static unsigned int windowCounter;
  
      private: friend class SingletonT<WindowManager>;
    };
  }
}
#endif
