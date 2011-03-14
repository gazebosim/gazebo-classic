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
#ifndef SHAPE_HH
#define SHAPE_HH

#include <string>
#include <iostream>

#include "common/XMLConfig.hh"
#include "common/Common.hh"

namespace gazebo
{
	namespace physics
{
  class Geom;

  /// \brief Base class for all shapes
  class Shape : public Common
  {
    /// \brief Constructor
    public: Shape(Geom *p);

    /// \brief Destructor
    public: virtual ~Shape();
  
    /// \brief Load the shape
    public: virtual void Load(XMLConfigNode *node) = 0;
  
    /// \brief Save the shape
    public: virtual void Save(std::string &prefix, std::ostream &stream) = 0;

    protected: Geom *geomParent;
  };
}

}
#endif
