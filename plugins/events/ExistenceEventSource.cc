/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "ExistenceEventSource.hh"

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
ExistenceEventSource::ExistenceEventSource(transport::PublisherPtr _pub,
                                           physics::WorldPtr _world)
  :EventSource(_pub, "existence", _world)
{
}

////////////////////////////////////////////////////////////////////////////////
void ExistenceEventSource::Load(const sdf::ElementPtr _sdf)
{
  EventSource::Load(_sdf);
  if (_sdf->HasElement("model"))
  {
    this->model = _sdf->GetElement("model")->Get<std::string>();
  }

  this->existenceConnection = SimEventConnector::ConnectSpawnModel(
      boost::bind(&ExistenceEventSource::OnExistence, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////
void ExistenceEventSource::OnExistence(const std::string &_model, bool _alive)
{
  // is this a model we're interested in?
  if (_model.find(this->model) == 0)
  {
    // set the data for the existence event
    std::string json = "{";
    json += "\"event\":\"existence\",";
    if (_alive)
    {
      json += "\"state\":\"creation\",";
    }
    else
    {
      json += "\"state\":\"deletion\",";
    }
    json += "\"model\":\"" + _model + "\"";
    json += "}";

    this->Emit(json);
  }
}
