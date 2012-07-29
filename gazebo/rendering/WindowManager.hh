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

#include <string>
#include <vector>

#include "common/SingletonT.hh"
#include "rendering/RenderTypes.hh"

namespace Ogre
{
  class RenderWindow;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{
    /// \brief Class to mangage render windows
    class WindowManager : public SingletonT<WindowManager>
    {
      public: WindowManager();
      public: virtual ~WindowManager();

      /// \brief Shutdown all the windows
      public: void Fini();

      public: int CreateWindow(const std::string &_ogreHandle,
                               unsigned int _width,
                               unsigned int _height);

      public: void GetAttribute(unsigned int _id,
                  const std::string &_attr, void *_data);

      /// \brief Attach a camera to a window
      public: void SetCamera(int _windowId, CameraPtr _camera);

      /// \brief Resize a window
      public: void Resize(unsigned int _id, int _width, int _height);

      public: void Moved(unsigned int _id);

      /// \brief Get the average FPS
      public: float GetAvgFPS(unsigned int _windowId);

      /// \brief Get the triangle count
      public: unsigned int GetTriangleCount(unsigned int _windotId);

      /// \brief Get the render window associated with _id
      public: Ogre::RenderWindow *GetWindow(unsigned int _id);

      private: std::vector<Ogre::RenderWindow *> windows;

      private: static unsigned int windowCounter;

      private: friend class SingletonT<WindowManager>;
    };
    /// \}
  }
}
#endif


