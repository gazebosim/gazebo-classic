#ifndef RENDERING_HH
#define RENDERING_HH

#include "RenderTypes.hh"

namespace gazebo
{
  namespace common
  {
    class XMLConfigNode;
  }

  namespace rendering
  {
    bool load(const std::string &filename);

    bool init();
    bool fini();

    rendering::ScenePtr create_scene(const std::string &name);
  }
}
#endif
