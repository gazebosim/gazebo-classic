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

#include "RestServicePlugin.hh"
#include <uuid/uuid.h>

using namespace gazebo;
using namespace std;


SimpleMOOCPlugin::SimpleMOOCPlugin()
  :node(new gazebo::transport::Node()),
  stopMsgProcessing(false),
  requestQThread(NULL)
{

  cout << "SimpleMOOCPlugin::SimpleMOOCPlugin()" << endl;
  // generate a unique session ID
  uuid_t uuid;
  uuid_generate_random ( uuid );
  char s[37];
  uuid_unparse ( uuid, s ); 

  this->session = s;

  cout << "Session : " << this->session << endl;
}

SimpleMOOCPlugin::~SimpleMOOCPlugin()
{
  node.reset();
  subEvent.reset();
  subRequest.reset();
  // tell the requestQ to stop precessing
  stopMsgProcessing = true;
  if(this->requestQThread->joinable()) {
    this->requestQThread->join();
    delete this->requestQThread;
  }
  cout << "SimpleMOOCPlugin::~SimpleMOOCPlugin()" << endl;
}

void SimpleMOOCPlugin::Init()
{
  std::cerr << "SimpleMOOCPlugin::Init() setting up pubs/sub node" <<  std::endl;
  // setup our node for communication
//  node->Init( string("default"));

  node->Init();
  cout << "   RestLogin subscription" << endl;
  subRequest = node->Subscribe("/gazebo/event/rest_login", &SimpleMOOCPlugin::OnRestLoginRequest, this);
  cout << "   RestPost subscription" << endl;
  subEvent = node->Subscribe("/gazebo/event/rest_post", &SimpleMOOCPlugin::OnEventRestPost, this);
  cout << "   starting request thread" << endl;
  requestQThread = new boost::thread( boost::bind(&SimpleMOOCPlugin::RunRequestQ, this));
}

void SimpleMOOCPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
  std::cerr << "Simple MOOC server plugin Load()" <<  std::endl;
}



////////////////////////////////////////////////////////////////////////////////
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


void SimpleMOOCPlugin::OnEventRestPost(ConstRestPostPtr &_msg)
{
  cout << "SimpleMOOCPlugin::OnRestPost";
  cout << "[" << _msg->route() << ", " << _msg->json() << "]"  << endl; 
  cout << endl;

  try
  {
   std::string event = "{";
    event += "\"event\": " + _msg->json() + ", ";
    physics::WorldPtr world = physics::get_world();
    if (!world) {
      cout << "!!!!! NO WORLD !!!" << endl;
    }
    else
    {
      event += "\"session\": \"" + this->session + "\", ";
      event += "\"world\": {";

      event += "\"name\": ";
      event += "\"";
      event += world->GetName();
      event += "\", ";


      if(!world->IsPaused() )
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
    std::string errorMsg ("There was a problem trying to send data to the MOOC: ");
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    cerr << "ERROR in request... publising to MOOCUI: " << errorMsg << endl;
    this->pub->Publish(msg);
  }

}

void SimpleMOOCPlugin::OnRestLoginRequest(ConstRestLoginPtr &_msg )
{
  cout << "SimpleMOOC received RestLogin message: [";
  cout << _msg->url() << ", ";
  cout << _msg->username() << ", ";
  cout << "*****"; // _msg->password();
  cout << "]";
  cout << endl;
  {
    boost::mutex::scoped_lock lock(this->requestQMutex);
    msgLoginQ.push_back(_msg);
  }

}


void SimpleMOOCPlugin::ProcessLoginRequest(ConstRestLoginPtr _msg)
{
 // this is executed asynchronously
 try
  {
    std::string resp;
    cout << "LOGIN " <<  _msg->username().c_str() << endl;
    resp = restApi.Login(_msg->url().c_str(), "/login", _msg->username().c_str(), _msg->password().c_str());
  }
  catch(RestException &x)
  {
    gazebo::msgs::RestError msg;
    std::string errorMsg ("There was a problem trying to login the web server: ");
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    cerr << "ERROR in request... publising to MOOCUI: " << errorMsg << endl;
    this->pub->Publish(msg);
  } 
}

void SimpleMOOCPlugin::ProcessMOOCEvent(ConstRestPostPtr _msg)
{
  // this is executed asynchronously
  try
  {
    restApi.PostJsonData(_msg->route().c_str(), _msg->json().c_str());
  }
  catch(RestException &x)
  {
    gazebo::msgs::RestError msg;
    std::string errorMsg ("There was a problem trying to post data to the web server: ");
    errorMsg += x.what();
    msg.set_type("Error");
    msg.set_msg(errorMsg);
    // alert the user via the gui plugin
    cerr << "ERROR POSTING to MOOC: " << errorMsg << endl;
    this->pub->Publish(msg);
  } 
}

void SimpleMOOCPlugin::RunRequestQ()
{
  
  // be ready to send errors back to the UI
  cout << "SimpleMOOCPlugin::RunRequestQ THREAD started" << endl;
  pub = node->Advertise<gazebo::msgs::RestError>("/gazebo/event/rest_error");

  // process any login or post data that ha been received
  while (!stopMsgProcessing) {
    gazebo::common::Time::MSleep(50);
    try{
      boost::shared_ptr<const gazebo::msgs::RestLogin> login;
      boost::shared_ptr<const gazebo::msgs::RestPost> post;
      // Grab the mutex and remove first message in each queue
      {
        boost::mutex::scoped_lock lock(this->requestQMutex);
        if(!msgLoginQ.empty())
        {
          login = msgLoginQ.front();
          msgLoginQ.pop_front();
        }
        if(!msgEventQ.empty())
        {
          post = msgEventQ.front();
          msgEventQ.pop_front();
        }
      }

      if(login)
      {
        this->ProcessLoginRequest(login);
      }
      if(post)
      {
        this->ProcessMOOCEvent(post);
      }
    }
    catch(...) {
      cerr << "Unhandled exception while processing request message" << endl;
    }
    
  }
  cout << "SimpleMOOCPlugin::RunRequestQ THREAD started" << endl;
}


// plugin registration
GZ_REGISTER_SYSTEM_PLUGIN(SimpleMOOCPlugin)

