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
/* Desc: Handles messages from the graphics ifaces
 * Author: Nate Koenig
 * Date: 9 Mar 2009
 * SVN: $Id:$
 */

#ifndef GRAPHICS_HH
#define GRAPHICS_HH

#include <map>
#include "gz.h"

namespace gazebo
{

  class Entity;
  class Visual;
  
  /// \brief Used to handle message from the graphics3d libgazebo iface
  class GraphicsIfaceHandler
  {
    /// \brief Constructor
    public: GraphicsIfaceHandler(World *world);
  
    /// \brief Destructor
    public: virtual ~GraphicsIfaceHandler();

    /// \brief Load the graphics handler
    public: void Load(const std::string &name, Entity *parent = NULL);

    /// \brief Init the graphics handler
    public: void Init();
  
    /// \brief Update the graphics handler
    public: void Update();

    /// \brief Helper funciton used to draw simple primitives
    private: void DrawSimple(Visual *vis, libgazebo::Graphics3dDrawData *data);

    /// \brief Helper funciton used to draw shapes
    private: void DrawShape(Visual *vis, libgazebo::Graphics3dDrawData *data);

    /// \brief Helper funciton used to draw text
    private: void DrawText(Visual *vis, libgazebo::Graphics3dDrawData *data);

    /// \brief Helper function used to draw a progress bar
    private: void DrawMeterBar(Visual *vis, libgazebo::Graphics3dDrawData *data );

    private: World *world;
    private: std::string name;
    private: std::map<std::string, Visual* > visuals;
    private: libgazebo::Graphics3dIface *threeDIface;
    private: Entity *parent;
  };

}

#endif
