/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifdef _WIN32
  #include <Rpc.h>
  #pragma comment(lib, "Rpcrt4.lib")
#else /* UNIX */
  #include <uuid/uuid.h>
#endif

#include "RestWebPlugin.hh"


using namespace gazebo;
using namespace std;

//////////////////////////////////////////////////
RestWebPlugin::RestWebPlugin()
: node(new gazebo::transport::Node()),
  stopMsgProcessing(false),
  requestQThread(NULL)
{
  // generate a unique session ID
  // On Windows
#ifdef _WIN32
  UUID uuid;
  C_STATUS Result = ::UuidCreate(uuid);
  if (Result != RPC_S_OK)
  {
    std::cerr << "Call to UuidCreate return a non success RPC call. " <<
                 "Return code: " << Result << std::endl;
  }
  char* szUuid = NULL;
  if (::UuidToStringA(&this->data, reinterpret_cast<RPC_CSTR*>(&szUuid)) ==
    RPC_S_OK)
  {
    this->session = szUuid;
    ::RpcStringFreeA(reinterpret_cast<RPC_CSTR*>(&szUuid));
  }
  // or on UNIX
#else
  uuid_t uuid;
  uuid_generate_random(uuid);
  char s[37];
  uuid_unparse(uuid, s);
  this->session = s;
#endif

  gzmsg << "REST web Session : " << this->session << endl;
}

//////////////////////////////////////////////////
RestWebPlugin::~RestWebPlugin()
{
  // tell the requestQ to stop precessing
  this->stopMsgProcessing = true;
  if (this->requestQThread->joinable())
  {
    this->requestQThread->join();
    delete this->requestQThread;
  }
}

//////////////////////////////////////////////////
void RestWebPlugin::Init()
{
  // setup our node for communication
  this->node->Init();
  this->subRequest = node->Subscribe("/gazebo/rest/rest_login",
                               &RestWebPlugin::OnRestLoginRequest, this);
  this->subEvent = node->Subscribe("/gazebo/rest/rest_post",
                             &RestWebPlugin::OnEventRestPost, this);
  this->subSimEvent = node->Subscribe("/gazebo/sim_events",
                                &RestWebPlugin::OnSimEvent, this);
  this->requestQThread = new boost::thread(
      boost::bind(&RestWebPlugin::RunRequestQ, this));
}

//////////////////////////////////////////////////
void RestWebPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  // nothing to do for now
}

//////////////////////////////////////////////////
std::string FormatTime(int _sec, int _nsec)
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _sec;

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_nsec * 1e-6);

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}

//////////////////////////////////////////////////
void RestWebPlugin::OnSimEvent(ConstSimEventPtr &_msg)
{
  // where to post the data on the REST server
  std::string route = "/events/new";

  std::string eType = _msg->type();
  std::string name = _msg->name();
  std::string data = _msg->data();

  msgs::WorldStatistics ws = _msg->world_statistics();
  msgs::Time simT = ws.sim_time();
  msgs::Time realT = ws.real_time();
  msgs::Time pauseT = ws.pause_time();
  bool paused = ws.paused();

  std::string worldName = physics::get_world()->GetName();
  std::string event = "{\n";

  event += "\"session\": \"" + this->session + "\", ";
  event += "\"name\": \"" + name + "\", ";
  event += "\"type\": \"" + eType + "\",\n";
  event += "\"data\": " + data + ", ";

  event += "\"world\": {";
  event += "\"name\": ";
  event += "\"";
  event += worldName;
  event += "\", ";

  event += "\"paused\": ";
  event += "\"";
  if (paused)
    event += "true";
  else
    event += "false";
  event += "\", ";

  event += "\"clock_time\": ";
  event += "\"";
  event += common::Time::GetWallTimeAsISOString();
  event += "\", ";

  event += "\"real_time\": ";
  event += "\"";
  event += FormatTime(realT.sec(), realT.nsec());
  event += "\", ";

  event += "\"sim_time\": ";
  event += "\"";
  event += FormatTime(simT.sec(), simT.nsec());
  event += "\", ";

  event += "\"pause_time\": ";
  event += "\"";
  event += FormatTime(pauseT.sec(), pauseT.nsec());
  event += "\"";

  event += "}\n";  // world element
  event += "}";    // root element

  // post it with curl
  this->restApi.PostJsonData(route.c_str(), event.c_str());
}

//////////////////////////////////////////////////
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
      event += FormatTime(t.sec, t.nsec);
      event += "\", ";

      event += "\"sim_time\": ";
      event += "\"";
      t = world->GetSimTime();
      event += FormatTime(t.sec, t.nsec);
      event += "\", ";

      event += "\"pause_time\": ";
      event += "\"";
      t = world->GetPauseTime();
      event += FormatTime(t.sec, t.nsec);
      event += "\" ";

      event += "}";
    }
    event += "}";
    this->restApi.PostJsonData(_msg->route().c_str(), event.c_str());
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
    gzerr << "ERROR in REST request: " << errorMsg << std::endl;
    this->pub->Publish(msg);
  }
}

//////////////////////////////////////////////////
void RestWebPlugin::OnRestLoginRequest(ConstRestLoginPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->requestQMutex);
  this->msgLoginQ.push_back(_msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::ProcessLoginRequest(ConstRestLoginPtr _msg)
{
  // this is executed asynchronously
  try
  {
    this->restApi.Login(_msg->url().c_str(),
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
    gzerr << "ERROR in REST login request. : " << errorMsg << std::endl;
    this->pub->Publish(msg);
  }
}

//////////////////////////////////////////////////
void RestWebPlugin::RunRequestQ()
{
  // be ready to send errors back to the UI
  std::string path("/gazebo/rest/rest_error");
  this->pub = node->Advertise<gazebo::msgs::RestError>(path);
  // process any login or post data that ha been received
  while (!this->stopMsgProcessing)
  {
    gazebo::common::Time::MSleep(50);
    try
    {
      boost::shared_ptr<const gazebo::msgs::RestLogin> login;
      // Grab the mutex and remove first message the queue
      {
        boost::mutex::scoped_lock lock(this->requestQMutex);
        if (!this->msgLoginQ.empty())
        {
          login = this->msgLoginQ.front();
          this->msgLoginQ.pop_front();
        }
      }
      if (login)
      {
        this->ProcessLoginRequest(login);
      }
    }
    catch(...)
    {
      gzerr << "Unhandled exception while processing request message"
          << std::endl;
    }
  }
}

// plugin registration
GZ_REGISTER_SYSTEM_PLUGIN(RestWebPlugin)
