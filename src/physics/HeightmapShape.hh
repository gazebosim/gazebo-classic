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
/* Desc: Heightmap shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 */

#ifndef HEIGHTMAPSHAPE_HH
#define HEIGHTMAPSHAPE_HH

#include "common/Image.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/Shape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \brief Height map geom
    class HeightmapShape : public Shape
    {
      /// \brief Constructor
      public: HeightmapShape(GeomPtr parent);
  
      /// \brief Destructor
      public: virtual ~HeightmapShape();
  
      /// \brief Update function 
      public: void Update();
  
      /// \brief Load the heightmap
      public: virtual void Load(common::XMLConfigNode *node);

      /// \brief Initialize the heightmap
      public: virtual void Init();
  
      /// \brief Save child parameters
      protected: void Save(std::string &prefix, std::ostream &stream);
  
      protected: common::Vector3 terrainSize;
  
      protected: std::vector<double> heights;
  
      protected: common::Image img;
      protected: common::ParamT<std::string> *imageFilenameP;
      protected: common::ParamT<std::string> *worldTextureP;
      protected: common::ParamT<std::string> *detailTextureP;
      protected: common::ParamT<common::Vector3> *sizeP;
      protected: common::ParamT<common::Vector3> *offsetP;
  
      // NATY: protected: OgreHeightmap *ogreHeightmap;
    };
  }

}
#endif
