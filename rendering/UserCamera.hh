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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 * SVN: $Id$
 */

#ifndef USERCAMERA_HH
#define USERCAMERA_HH

#include "XMLConfig.hh"
#include "Camera.hh"

namespace Ogre
{
  class RenderWindow;
  class AnimationState;
}

namespace gazebo
{
  class RenderControl;
  class XMLConfigNode;
  class Visual;

  class UserCamera : public Camera
  {
    /// \brief Constructor
    public: UserCamera( const std::string &name, Scene *scene);

    /// \brief Destructor
    public: virtual ~UserCamera();

    /// \brief Load child
    public: void Load( XMLConfigNode *node );

    /// \brief Initialize
    public: void Init();

    /// \brief Render the camera
    public: virtual void Render();

    /// \brief Post render
    public: virtual void PostRender();

    /// \brief Finialize
    public: void Fini();

    /// \brief Hande a mouse event
    public: void HandleMouseEvent(const MouseEvent &evt);

    /// \brief Set view controller
    public: void SetViewController( const std::string type );

    /// \brief Resize the camera
    //public: void Resize(unsigned int w, unsigned int h);

    /// \brief Set the dimensions of the viewport
    public: void SetViewportDimensions(float x, float y, float w, float h);

    /// \brief Get the average FPS
    public: virtual float GetAvgFPS();

    /// \brief Get the triangle count
    public: unsigned int GetTriangleCount();

    /// \brief Get the ogre window
    public: Ogre::RenderWindow *GetWindow();

    /// \brief Move the camera to focus on an entity
    public: void MoveToEntity(Entity *entity);

    /// \brief Set the camera to track an entity
    public: void TrackModel( Model *model );
 
    /// \brief Toggle whether to show the visual
    private: void ToggleShowVisual();

    /// \brief Set whether to show the visual
    private: void ShowVisual(bool s);

    /// Pointer to the render window
    private: Ogre::RenderWindow *window;

    private: std::string name;
    private: static unsigned int cameraCount;
    private: static int count;

    private: Visual *visual;

    private: ViewController *viewController;

    private: std::vector<event::ConnectionPtr> connections;
    private: Ogre::AnimationState *animState;
  };
}

#endif
