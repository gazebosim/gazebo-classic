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

EventT<void (bool)> Events::pauseSignal;
EventT<void ()> Events::stepSignal;
EventT<void ()> Events::quitSignal;

EventT<void (bool)> Events::moveModeSignal;
EventT<void (bool)> Events::manipModeSignal;

EventT<void (std::string)> Events::createEntitySignal;
EventT<void (std::string)> Events::setSelectedEntitySignal;
EventT<void (std::string)> Events::addEntitySignal;
EventT<void (std::string)> Events::deleteEntitySignal;
EventT<void (bool)> Events::showLightsSignal;
EventT<void (bool)> Events::showCamerasSignal;
EventT<void (bool)> Events::showContactsSignal;
EventT<void (bool)> Events::wireframeSignal;
EventT<void (bool)> Events::showPhysicsSignal;
EventT<void (bool)> Events::showJointsSignal;
EventT<void (bool)> Events::showBoundingBoxesSignal;
EventT<void (Entity*)> Events::entitySelectedSignal;

EventT<void ()> Events::worldUpdateStartSignal;
EventT<void ()> Events::worldUpdateEndSignal;

EventT<void ()> Events::preRenderSignal;
EventT<void ()> Events::renderSignal;
EventT<void ()> Events::postRenderSignal;

EventT<void (std::string)> Events::diagTimerStartSignal;
EventT<void (std::string)> Events::diagTimerStopSignal;

