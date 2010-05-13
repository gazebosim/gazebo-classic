#include "Events.hh"

using namespace gazebo;

boost::signal<void (std::string)> Events::createEntitySignal;
