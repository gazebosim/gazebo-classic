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

#include <gazebo/gazebo.hh>

#ifdef HAVE_UUID
  #include <uuid/uuid.h>
#endif

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
    gzerr << "Call to UuidCreate return a non success RPC call. " <<
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

#ifdef HAVE_UUID
  uuid_t uuid;
  uuid_generate_random(uuid);
  char s[37];
  uuid_unparse(uuid, s);
  this->session = s;
#endif

#endif
  if (this->session.empty())
  {
    // alternative to uuid
    this->session = common::Time::GetWallTimeAsISOString();
  }
  gzmsg << "REST web Session : " << this->session << endl;
}

//////////////////////////////////////////////////
RestWebPlugin::~RestWebPlugin()
{
  // tell the requestQ to stop precessing
  this->stopMsgProcessing = true;
  if (this->requestQThread && this->requestQThread->joinable())
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
  this->subLogin = node->Subscribe("/gazebo/rest/rest_login",
                               &RestWebPlugin::OnRestLoginRequest, this);

  this->subLogout = node->Subscribe("/gazebo/rest/rest_logout",
                               &RestWebPlugin::OnRestLogoutRequest, this);

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
void RestWebPlugin::OnSimEvent(ConstSimEventPtr &_msg)
{
  gazebo::msgs::RestResponse msg;
  std::string response;
  try
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
    event += msgs::Convert(realT).FormattedString();
    event += "\", ";

    event += "\"sim_time\": ";
    event += "\"";
    event += msgs::Convert(simT).FormattedString();
    event += "\", ";

    event += "\"pause_time\": ";
    event += "\"";
    event += msgs::Convert(pauseT).FormattedString();
    event += "\"";

    event += "}\n";  // world element
    event += "}";    // root element
    // post it with curl
    this->restApi.PostJsonData(route.c_str(), event.c_str());
    msg.set_type(msgs::RestResponse::SUCCESS);
  }
  catch(RestException &x)
  {
    response = "There was a problem trying to send data to the server: ";
    response += x.what();
    msg.set_type(msgs::RestResponse::ERROR);
    // alert the user via the gui plugin
    gzerr << "ERROR in REST service POST request: " << response << std::endl;
  }

  msg.set_msg(response);
  this->pub->Publish(msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::OnEventRestPost(ConstRestPostPtr &_msg)
{
  gzmsg << "RestWebPlugin::OnRestPost";
  gzmsg << "[" << _msg->route() << ", " << _msg->json() << "]"  << std::endl;
  gzmsg << std::endl;

  gazebo::msgs::RestResponse msg;
  std::string response;
  try
  {
    std::string event = "{";
    event += "\"event\": " + _msg->json() + ", ";
    physics::WorldPtr world = physics::get_world();
    if (!world)
    {
      gzerr << "Can't access world before web service POST" << std::endl;
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
      event += t.FormattedString();
      event += "\", ";

      event += "\"sim_time\": ";
      event += "\"";
      t = world->GetSimTime();
      event += t.FormattedString();
      event += "\", ";

      event += "\"pause_time\": ";
      event += "\"";
      t = world->GetPauseTime();
      event += t.FormattedString();
      event += "\" ";

      event += "}";
    }
    event += "}";

    this->restApi.PostJsonData(_msg->route().c_str(), event.c_str());
    msg.set_type(msgs::RestResponse::SUCCESS);
  }
  catch(RestException &x)
  {
    response = "There was a problem trying to send data to the server: ";
    response += x.what();
    msg.set_type(msgs::RestResponse::ERROR);
    // alert the user via the gui plugin
    gzerr << "ERROR in REST request: " << response << std::endl;
  }

  if (_msg->has_id())
    msg.set_id(_msg->id());
  msg.set_msg(response);
  this->pub->Publish(msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::OnRestLoginRequest(ConstRestLoginPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->requestQMutex);
  this->msgLoginQ.push_back(_msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::OnRestLogoutRequest(ConstRestLogoutPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->requestQMutex);
  this->restApi.Logout();

  gazebo::msgs::RestResponse msg;
  if (_msg->has_id())
    msg.set_id(_msg->id());
  msg.set_type(msgs::RestResponse::LOGOUT);
  msg.set_msg("Success");
  this->pub->Publish(msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::ProcessLoginRequest(ConstRestLoginPtr _msg)
{
  gazebo::msgs::RestResponse msg;
  std::string response;

  // this is executed asynchronously
  try
  {
    this->restApi.Login(_msg->url().c_str(),
        "/login",
        _msg->username().c_str(),
        _msg->password().c_str());

    response = "Success";
    msg.set_type(msgs::RestResponse::LOGIN);
  }
  catch(RestException &x)
  {
    response = "There was a problem trying to login to the server: ";
    response += x.what();
    msg.set_type(msgs::RestResponse::ERROR);

    // alert the user via the gui plugin
    gzerr << "ERROR in REST login request. : " << response << std::endl;
  }

  if (_msg->has_id())
    msg.set_id(_msg->id());
  msg.set_msg(response);
  this->pub->Publish(msg);
}

//////////////////////////////////////////////////
void RestWebPlugin::RunRequestQ()
{
  // be ready to send errors back to the UI
  std::string path("/gazebo/rest/rest_response");
  this->pub = node->Advertise<gazebo::msgs::RestResponse>(path);
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
