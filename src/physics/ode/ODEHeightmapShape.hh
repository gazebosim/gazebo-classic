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
/* Desc: ODE Heightmap shape
 * Author: Nate Keonig
 * Date: 12 Nov 2009
 * SVN: $Id$
 */

#ifndef ODEHEIGHTMAPSHAPE_HH
#define ODEHEIGHTMAPSHAPE_HH

#include "HeightmapShape.hh"
#include "Vector2.hh"
#include "ODEPhysics.hh"
#include "Geom.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief ODE Height map geom
    class ODEHeightmapShape : public HeightmapShape
    {
      /// \brief Constructor
      public: ODEHeightmapShape(Geom *parent);
  
      /// \brief Destructor
      public: virtual ~ODEHeightmapShape();
  
      /// \brief Update function 
      public: void Update();
  
      /// \brief Load the heightmap
      protected: virtual void Load(XMLConfigNode *node);
  
      /// Create a lookup table of the terrain's height
      private: void FillHeightMap();
  
      /// \brief Called by ODE to get the height at a vertex
      private: static dReal GetHeightCallback(void *data, int x, int y);
  
      private: dHeightfieldDataID odeData;
  
      private: unsigned int odeVertSize;
      private: Vector3 odeScale;
  
      private: std::vector<double> heights;
    };
  
    /// \}
  }
}
#endif
