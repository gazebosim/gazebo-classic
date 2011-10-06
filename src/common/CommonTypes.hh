#ifndef GAZEBO_COMMON_TYPES_HH
#define GAZEBO_COMMON_TYPES_HH

#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_common
/// \brief Forward declarations for the common classes
namespace gazebo
{
  class WorldPlugin;
  class ModelPlugin;
  class SensorPlugin;
  class GUIPlugin;

  typedef boost::shared_ptr<WorldPlugin> WorldPluginPtr;
  typedef boost::shared_ptr<ModelPlugin> ModelPluginPtr;
  typedef boost::shared_ptr<SensorPlugin> SensorPluginPtr;
  typedef boost::shared_ptr<GUIPlugin> GUIPluginPtr;

  namespace common
  {
    class Param;
    class Time;
    class Image;
    class Mesh;
    class MouseEvent;


    template <typename T>
    class ParamT;

    typedef std::vector<common::Param*> Param_V;
    typedef std::map<std::string, std::string> StrStr_M;
  }

  namespace event
  {
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;
    typedef std::vector<ConnectionPtr> Connection_V;
  }
}

#endif
