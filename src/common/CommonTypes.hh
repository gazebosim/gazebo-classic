#ifndef GAZEBO_COMMON_TYPES_HH
#define GAZEBO_COMMON_TYPES_HH

#include <vector>
#include <map>
#include <string>

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

    template <typename T>
    class ParamT;

    typedef std::vector<common::Param*> Param_V;
    typedef std::map<std::string, std::string> StrStr_M;
  }
}

#endif
