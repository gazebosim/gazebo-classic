#ifndef RENDERING_HH
#define RENDERING_HH

enum OperationType 
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
};

#endif
