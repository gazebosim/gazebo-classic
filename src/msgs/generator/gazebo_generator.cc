#include <google/protobuf/compiler/plugin.h>
#include "GazeboGenerator.hh"

int main(int _argc, char *_argv[])
{
  #ifdef _MSC_VER
  // Don't print a silly message or stick a modal dialog box in my face,
  // please.
  _set_abort_behavior(0, ~0);
#endif  // !_MSC_VER

  google::protobuf::compiler::cpp::GazeboGenerator generator("gazebo_plugin");
  return google::protobuf::compiler::PluginMain(_argc, _argv, &generator);
}

