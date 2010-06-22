#include "gazebo_config.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <list>

#include "Simulator.hh"
#include "GazeboConfig.hh"
#include "GazeboError.hh"
#include "Plugin.hh"

#ifdef HAVE_DL
#include <dlfcn.h>
#elif HAVE_LTDL
#include <ltdl.h>
#endif


using namespace gazebo;

Plugin::Plugin() {}
Plugin::~Plugin() {}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of the handler
std::string Plugin::GetFilename() const
{
  return this->filename;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the short name of the handler
std::string Plugin::GetHandle() const
{
  return this->handle;
}

Plugin *Plugin::Create(const std::string &filename, const std::string &shortname)
{
  Plugin *result = NULL;
  struct stat st;
  bool found = false;
  std::string fullname;
  std::list<std::string>::iterator iter;
  std::list<std::string> pluginPaths= Simulator::Instance()->GetGazeboConfig()->GetPluginPaths();

  for (iter=pluginPaths.begin(); iter!=pluginPaths.end(); ++iter)
  {
    fullname = (*iter)+std::string("/")+filename;
    if (stat(fullname.c_str(), &st) == 0) {found=true; break;}
  }
  if (!found) fullname = filename;
  std::string registerName = "RegisterPlugin";

#ifdef HAVE_DL
  void* handle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
  if (!handle)
  {
    std::cerr << "Failed to load plugin " << fullname << ": " << dlerror();
    return NULL;
  }

  Plugin *(*registerFunc)() = (Plugin *(*)())dlsym(handle, registerName.c_str());
  if(!registerFunc)
  {
    std::cerr << "Failed to resolve " << registerName << ": " << dlerror();
    return NULL;
  }

  // Register the new controller.
  result = registerFunc();

#elif HAVE_LTDL

  static bool init_done = false;

  if (!init_done)
  {
    int errors = lt_dlinit();
    if (errors)
    {
      std::cerr << "Error(s) initializing dynamic loader (" 
        << errors << ", " << lt_dlerror() << ")";
      return NULL;
    }
    else
      init_done = true;
  }

  lt_dlhandle handle = lt_dlopenext(fullname.c_str());

  if (!handle)
  {
    std::cerr << "Failed to load " << fullname << ": " << lt_dlerror();
    return NULL;
  }

  Plugin *(*registerFunc)() = (Plugin *(*)())lt_dlsym(handle, registerName.c_str());
  if(!registerFunc)
  {
    std::cerr << "Failed to resolve " << registerName << ": " << lt_dlerror();
    return NULL;
  }

  // Register the new controller.
  result = registerFunc();

#else // HAVE_LTDL

  gzthrow("Cannot load plugins as libtool is not installed.");

#endif // HAVE_LTDL

  result->handle = shortname;
  result->filename = filename;

  return result;
}
