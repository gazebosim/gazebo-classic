/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Render state container
 * Author: Nate Koenig
 * Date: 06 Oct 2010
 */

#ifndef RENDERSTATE_HH
#define RENDERSTATE_HH

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
  };
}
#endif
