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

#include <string.h>
#include <math.h>

#include "common/Image.hh"
#include "common/Exception.hh"

#include "physics/HeightmapShape.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Constructor
HeightmapShape::HeightmapShape(CollisionPtr parent)
    : Shape(parent)
{
  this->AddType(Base::HEIGHTMAP_SHAPE);
  // NATY: this->ogreHeightmap = new OgreHeightmap(this->GetWorld()->GetScene());
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
HeightmapShape::~HeightmapShape()
{
  // NATY: delete this->ogreHeightmap;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void HeightmapShape::Update()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void HeightmapShape::Load( sdf::ElementPtr &_sdf )
{
  Shape::Load(_sdf);

  // Use the image to get the size of the heightmap
  this->img.Load( this->sdf->GetValueString("filename") );

  // Width and height must be the same
  if (this->img.GetWidth() != this->img.GetHeight())
    gzthrow("Heightmap image must be square\n");
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the heightmap
void HeightmapShape::Init()
{
}
