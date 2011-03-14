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
#ifndef RENDERING_HH
#define RENDERING_HH

enum RenderOpType 
{
  /// A list of points, 1 vertex per point
  RENDERING_POINT_LIST = 0,//Ogre::RenderOperation::OT_POINT_LIST,

  /// A list of lines, 2 vertices per line
  RENDERING_LINE_LIST = 1,//Ogre::RenderOperation::OT_LINE_LIST,

  /// A strip of connected lines, 1 vertex per line plus 1 start vertex
  RENDERING_LINE_STRIP = 2,//Ogre::RenderOperation::OT_LINE_STRIP,

  /// A list of triangles, 3 vertices per triangle
  RENDERING_TRIANGLE_LIST = 3,//Ogre::RenderOperation::OT_TRIANGLE_LIST,

  /// A strip of triangles, 3 vertices for the first triangle, and 1 per triangle after that 
  RENDERING_TRIANGLE_STRIP = 4,//Ogre::RenderOperation::OT_TRIANGLE_STRIP,

  /// A fan of triangles, 3 vertices for the first triangle, and 1 per triangle after that
  RENDERING_TRIANGLE_FAN = 5,//Ogre::RenderOperation::OT_TRIANGLE_FAN 

  RENDERING_MESH_RESOURCE = 6
};

}
#endif
