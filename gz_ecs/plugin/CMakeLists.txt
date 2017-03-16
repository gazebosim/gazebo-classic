set(sources
  PluginLoader.cc
  PluginLoaderPrivate.cc
)

add_library(GazeboPlugin SHARED ${sources})
target_link_libraries(GazeboPlugin dl)
