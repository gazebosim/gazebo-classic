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
/* Desc: Render state container
 * Author: Nate Koenig
 * Date: 06 Oct 2010
 */

#ifndef RENDERSTATE_HH
#define RENDERSTATE_HH

#include "Event.hh"

namespace gazebo
{
  class RenderState 
  {
    private: RenderState();
    private: virtual ~RenderState();
    public: static void Init();

    public: static bool GetShowLights();
    public: static bool GetShowJoints();
    public: static bool GetShowCameras();
    public: static bool GetShowContacts();
    public: static bool GetShowWireframe();
    public: static bool GetShowPhysics();
    public: static bool GetShowBoundingBoxes();

    private: void ShowLightsCB();
    private: void ShowJointsCB();
    private: void ShowCamerasCB();
    private: void ShowContactsCB();
    private: void ShowWireframeCB();
    private: void ShowPhysicsCB();
    private: void ShowBoundingBoxesCB();

    private: static bool showLights;
    private: static bool showJoints;
    private: static bool showCameras;
    private: static bool showContacts;
    private: static bool showWireframe;
    private: static bool showPhysics;
    private: static bool showBoundingBoxes;

    private: static RenderState *self;
    private: std::vector<event::ConnectionPtr> connections;
  };
}
#endif
