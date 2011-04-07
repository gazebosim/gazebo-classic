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
    bool load(common::XMLConfigNode *node);

    bool init(bool create_dummy_window);

    rendering::ScenePtr create_scene(const std::string &name);

  }
}
#endif
