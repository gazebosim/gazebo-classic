#ifndef GAZEBO_COMMON_TYPES_HH
#define GAZEBO_COMMON_TYPES_HH

#include "common/Console.hh"
#include "common/Exception.hh"

namespace gazebo
{
  namespace common
  {
    class Param;
    class XMLConfigNode;
    class Pose3d;
    class Vector3;
    class Quaternion;
    class Box;
    class Time;
    class Image;

    typedef std::vector<common::Param*> Param_V;
  }
}

#endif
