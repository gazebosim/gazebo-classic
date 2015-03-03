/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <uuid/uuid.h>
#include "RestWebPlugin.hh"

using namespace gazebo;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
RestWebPlugin::RestWebPlugin()
  :node(new gazebo::transport::Node()),
  stopMsgProcessing(false),
  requestQThread(NULL)
{
  // generate a unique session ID
  uuid_t uuid;
  uuid_generate_random(uuid);
  char s[37];
  uuid_unparse(uuid, s);
  this->session = s;
  gzmsg << "REST web Session : " << this->session << endl;
}

////////////////////////////////////////////////////////////////////////////////
RestWebPlugin::~RestWebPlugin()
{
  node.reset();
  subEvent.reset();
  subRequest.reset();
  // tell the requestQ to stop precessing
  stopMsgProcessing = true;
  if (this->requestQThread->joinable())
  {
    this->requestQThread->join();
    delete this->requestQThread;
  }
}

////////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::Init()
{
  std::cerr << "RestWebPlugin::Init() setting up pubs/sub node" <<  std::endl;
  // setup our node for communication
  node->Init();
  subRequest = node->Subscribe("/gazebo/event/rest_login",
                               &RestWebPlugin::OnRestLoginRequest, this);
  subEvent = node->Subscribe("/gazebo/event/rest_post",
                             &RestWebPlugin::OnEventRestPost, this);
  subSimEvent = node->Subscribe("/gazebo/sim_events",
                                &RestWebPlugin::OnSimEvent, this);
  requestQThread = new boost::thread(boost::bind(&RestWebPlugin::RunRequestQ,
                                                  this));
}

////////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  // npthing to do for now
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::OnSimEvent(ConstSimEventPtr &_msg)
{
  gzmsg << "SIM EVENT\n";
  gzmsg << " type: " << _msg->type() << std::endl;
  gzmsg << " name: " << _msg->name() << std::endl;
  gzmsg << " data: " << _msg->data() << std::endl;

  msgs::WorldStatistics ws = _msg->world_statistics();
  
}

///////////////////////////////////////////////////////////////////////////////
//  adapted from TimePanel
std::string FormatTime(common::Time &_t)
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _t.sec;

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_t.nsec * 1e-6);

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::OnEventRestPost(ConstRestPostPtr &_msg)
{
  gzmsg << "RestWebPlugin::OnRestPost";
  gzmsg << "[" << _msg->route() << ", " << _msg->json() << "]"  << std::endl;
  gzmsg << std::endl;

  try
  {
    std::string event = "{";
    event += "\"event\": " + _msg->json() + ", ";
    physics::WorldPtr world = physics::get_world();
    if (!world)
    {
      gzerr << "World is null" << std::endl;
    }
    else
    {
      event += "\"session\": \"" + this->session + "\", ";
      event += "\"world\": {";

      event += "\"name\": ";
      event += "\"";
      event += world->GetName();
      event += "\", ";


      if (!world->IsPaused())
      {
        event += "\"is_running\": \"true\", ";
      }
      else
      {
        event +=  "\"is_running\": \"false\", ";
      }
      common::Time t;

      event += "\"clock_time\": ";
      event += "\"";
      event += common::Time::GetWallTimeAsISOString();
      event += "\", ";

      event += "\"real_time\": ";
      event += "\"";
      t = world->GetRealTime();
      event += FormatTime(t);
      event += "\", ";

      event += "\"sim_time\": ";
      event += "\"";
      t = world->GetSimTime();
      event += FormatTime(t);
      event += "\", ";

      event += "\"pause_time\": ";
      event += "\"";
      t = world->GetPauseTime();
      event += FormatTime(t);
      event += "\" ";

      event += "}";
    }
    event += "}";
    restApi.PostJsonData(_msg->route().c_str(), event.c_str());
  }
  catch(RestException &x)
  {
    gazebo::msgs::RestError msg;
    std::string errorMsg;
    errorMsg = "There was a problem trying to send data to the server: ";
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    cerr << "ERROR in REST request: " << errorMsg << std::endl;
    this->pub->Publish(msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::OnRestLoginRequest(ConstRestLoginPtr &_msg )
{
  boost::mutex::scoped_lock lock(this->requestQMutex);
  msgLoginQ.push_back(_msg);
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::ProcessLoginRequest(ConstRestLoginPtr _msg)
{
  // this is executed asynchronously
  try
  {
    restApi.Login(_msg->url().c_str(),
                  "/login",
                  _msg->username().c_str(),
                  _msg->password().c_str());
  }
  catch(RestException &x)
  {
    gazebo::msgs::RestError msg;
    std::string errorMsg;
    errorMsg = "There was a problem trying to login the server: ";
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    cerr << "ERROR in REST request. : " << errorMsg << std::endl;
    this->pub->Publish(msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::ProcessRestPostEvent(ConstRestPostPtr _msg)
{
  // this is executed asynchronously
  try
  {
    restApi.PostJsonData(_msg->route().c_str(), _msg->json().c_str());
  }
  catch(RestException &x)
  {
    gazebo::msgs::RestError msg;
    std::string errorMsg;
    errorMsg = "There was a problem trying to post data to the web server: ";
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    gzerr << "Error posting to REST server: " << errorMsg << std::endl;
    this->pub->Publish(msg);
  }
}

///////////////////////////////////////////////////////////////////////////////
void RestWebPlugin::RunRequestQ()
{
  // be ready to send errors back to the UI
  pub = node->Advertise<gazebo::msgs::RestError>("/gazebo/event/rest_error");
  // process any login or post data that ha been received
  while (!stopMsgProcessing)
  {
    gazebo::common::Time::MSleep(50);
    try
    {
      boost::shared_ptr<const gazebo::msgs::RestLogin> login;
      boost::shared_ptr<const gazebo::msgs::RestPost> post;
      // Grab the mutex and remove first message in each queue
      {
        boost::mutex::scoped_lock lock(this->requestQMutex);
        if (!msgLoginQ.empty())
        {
          login = msgLoginQ.front();
          msgLoginQ.pop_front();
        }
        if (!msgEventQ.empty())
        {
          post = msgEventQ.front();
          msgEventQ.pop_front();
        }
      }
      if (login)
      {
        this->ProcessLoginRequest(login);
      }
      if (post)
      {
        this->ProcessRestPostEvent(post);
      }
    }
    catch(...)
    {
      gzerr << "Unhandled exception while processing request message";
      gzerr << std::endl;
    }
  }
}


// plugin registration
GZ_REGISTER_SYSTEM_PLUGIN(RestWebPlugin)

