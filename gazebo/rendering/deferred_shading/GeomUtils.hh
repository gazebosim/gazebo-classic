/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _GEOMUTILS_HH_
#define _GEOMUTILS_HH_

#include <OgreString.h>
#include <OgreVertexIndexData.h>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class GAZEBO_VISIBLE GeomUtils
    {
      /// \brief Create a sphere Mesh with a given name, radius, number of
      /// rings and number of segments
      public: static void CreateSphere(const Ogre::String &_strName,
                  float _radius, int _nRings, int _nSegments, bool _bNormals,
                  bool _bTexCoords);

      /// \brief Fill up a fresh copy of VertexData and IndexData with a
      /// sphere's coords given the number of rings and the number of segments
      public: static void CreateSphere(Ogre::VertexData *&_vertexData,
                Ogre::IndexData *&_indexData, float _radius, int _nRings,
                int _nSegments, bool _bNormals, bool _bTexCoords);

      /// \brief Create a cone Mesh with a given name, radius and number of
      /// vertices in base Created cone will have its head at 0,0,0, and will
      /// 'expand to' positive y
      public: static void CreateCone(const Ogre::String &_strName,
                  float _radius, float _height, int _nVerticesInBase);

      /// \brief Fill up a fresh copy of VertexData and IndexData with a
      /// cone's coords given the radius and number of vertices in base
      public: static void CreateCone(Ogre::VertexData *&_vertexData,
                                     Ogre::IndexData *&_indexData,
                                     float _radius, float _height,
                                     int _nVerticesInBase);

      /// \brief Fill up a fresh copy of VertexData with a normalized quad
      public: static void CreateQuad(Ogre::VertexData *&_vertexData);
    };
  }
}
#endif
