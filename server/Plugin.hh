#ifndef HANDLER_HH
#define HANDLER_HH

#include <string>
#include <boost/signals.hpp>

namespace gazebo
{
  class Event;

  class Plugin : public boost::signals::trackable
  {
    public: Plugin();
    public: virtual ~Plugin();
    public: virtual void Load() = 0;

    /// \brief Get the name of the handler
    public: std::string GetFilename() const;

    /// \brief Get the short name of the handler
    public: std::string GetHandle() const;

    public: static Plugin *Create(const std::string &filename, const std::string &handle);

    protected: std::string filename;
    protected: std::string handle;
  };
}

#define GZ_REGISTER_PLUGIN(name, classname) \
extern "C" Plugin *RegisterPlugin(); \
Plugin *RegisterPlugin() \
{\
  return new classname();\
}
#endif
