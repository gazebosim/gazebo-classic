
/*
 * copyright (C) 2014 Open Source Robotics Foundation
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


#ifndef _REST_API_HH_
#define _REST_API_HH_

#include <list>
#include "RestException.hh"

namespace gazebo
{

  class RestApi
  {
    /// \brief ctor
    public: RestApi();

    /// \brief dtor
    public: virtual ~RestApi();

    /// \brief Connects to the Rest service. 
    /// \param[in] _url the web service url
    /// \param[in] _user the user name
    /// \param[in] _pass the user password
    /// \returns message from the REST server
    public: std::string Login(const char* _url, const char* _route, const char* _user, const char* _pass);

    /// \brief Notify the service with a http POST
    /// \param[in] _route on the web server
    /// \param[in] _json the data to send to the server
    public: void PostJsonData(const char* _route, const char *_json);
   
    /// \brief a Request/Response
    /// \param[in] the request

    /// \return the response
    private: std::string Request(const char* _request, const char *_postStr = NULL);
    
    /// \brief Login information: Rest service host url
    private: std::string url;

    /// \brief Login information: Rest service username 
    private: std::string user;

    /// \brief Login information: Rest service password
    private: std::string pass;

    /// \brief Login information: login route
    private: std::string loginRoute;

    /// \brief True when a previous Login attempt was successful
    private: bool isLoggedIn;

    /// \brief A post: what (json) and where (route)
    private: struct Post 
      {
        std::string route;
        std::string json;
      };

    /// \brief List of unposted posts. Posts await when isLoggedIn is false
    private: std::list<Post> posts;

    /// \brief Sends unposted posts
    private: void SendUnpostedPosts();
  };
}

#endif
