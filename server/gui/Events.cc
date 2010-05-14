#include "Events.hh"

using namespace gazebo;

boost::signal<void (std::string)> Events::createEntitySignal;
boost::signal<void (bool)> Events::moveModeSignal;
boost::signal<void (bool)> Events::manipModeSignal;
