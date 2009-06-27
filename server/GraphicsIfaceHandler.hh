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
/* Desc: Handles messages from the graphics ifaces
 * Author: Nate Koenig
 * Date: 9 Mar 2009
 * SVN: $Id:$
 */

#ifndef GRAPHICS_HH
#define GRAPHICS_HH

#include <map>
#include "gazebo.h"

namespace gazebo
{

  class Graphics3dIface;
  class Entity;
  class OgreVisual;
  
  /// \brief Used to handle message from the graphics3d libgazebo iface
  class GraphicsIfaceHandler
  {
    /// \brief Constructor
    public: GraphicsIfaceHandler();
  
    /// \brief Destructor
    public: virtual ~GraphicsIfaceHandler();

    /// \brief Load the graphics handler
    public: void Load(const std::string &name, Entity *parent = NULL);

    /// \brief Init the graphics handler
    public: void Init();
  
    /// \brief Update the graphics handler
    public: void Update();

    /// \brief Helper funciton used to draw simple primitives
    private: void DrawSimple(OgreVisual *vis, Graphics3dDrawData *data);

    /// \brief Helper funciton used to draw shapes
    private: void DrawShape(OgreVisual *vis, Graphics3dDrawData *data);

    /// \brief Helper funciton used to draw text
    private: void DrawText(OgreVisual *vis, Graphics3dDrawData *data);

    /// \brief Helper function used to draw a progress bar
    private: void DrawMeterBar(OgreVisual *vis, Graphics3dDrawData *data );

    private: std::string name;
    private: std::map<std::string, OgreVisual* > visuals;
    private: Graphics3dIface *threeDIface;
    private: Entity *parent;
  };

}

#endif
