#include "gazebo/plugin/RegisterMacros.hh"
#include "gazebo/test/util/DummyPlugins.hh"


namespace gazebo
{
namespace test
{
namespace util
{

std::string DummyPlugin::MyNameIs()
{
  return std::string("DummyPlugin");
}

}
}
}

GZ_REGISTER_SINGLE_PLUGIN(gazebo::test::util::DummyPlugin,
                          gazebo::test::util::DummyPluginBase);
