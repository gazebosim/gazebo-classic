#include "Events.hh"

using namespace gazebo;

boost::signal<void (bool)> Events::moveModeSignal;
boost::signal<void (bool)> Events::manipModeSignal;

boost::signal<void (std::string)> Events::createEntitySignal;
boost::signal<void (std::string)> Events::setSelectedEntitySignal;
boost::signal<void (std::string)> Events::addEntitySignal;
boost::signal<void (std::string)> Events::deleteEntitySignal;

