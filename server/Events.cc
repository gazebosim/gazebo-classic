/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "Events.hh"

using namespace gazebo;

boost::signal<void (bool)> Events::pauseSignal;
boost::signal<void ()> Events::stepSignal;
boost::signal<void ()> Events::quitSignal;

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

boost::signal<void ()> Events::preRenderSignal;
boost::signal<void ()> Events::renderSignal;
boost::signal<void ()> Events::postRenderSignal;


