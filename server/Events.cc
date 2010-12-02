#include "Events.hh"

using namespace gazebo;

boost::signal<void (bool)> Events::moveModeSignal;
boost::signal<void (bool)> Events::manipModeSignal;

boost::signal<void (std::string)> Events::createEntitySignal;
boost::signal<void (std::string)> Events::setSelectedEntitySignal;
boost::signal<void (std::string)> Events::addEntitySignal;
boost::signal<void (std::string)> Events::deleteEntitySignal;
boost::signal<void ()> Events::showLightsSignal;
boost::signal<void ()> Events::showCamerasSignal;
boost::signal<void ()> Events::showContactsSignal;
boost::signal<void ()> Events::wireframeSignal;
boost::signal<void ()> Events::showPhysicsSignal;
boost::signal<void ()> Events::showJointsSignal;
boost::signal<void ()> Events::showBoundingBoxesSignal;
boost::signal<void (Entity*)> Events::entitySelectedSignal;

boost::signal<void ()> Events::worldUpdateStartSignal;
boost::signal<void ()> Events::worldUpdateEndSignal;
boost::signal<void ()> Events::renderStartSignal;


