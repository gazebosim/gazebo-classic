/*
 * copyright (C) 2015 Open Source Robotics Foundation
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

#include <string>
#include <list>
#include <gazebo/common/Console.hh>

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
    /// \param[in] _url The web service url
    /// \param[in] _route The route on the server
    /// \param[in] _user The user name
    /// \param[in] _pass The user password
    /// \return The response message from the REST server
    public: std::string Login(const std::string &_url,
                              const std::string &_route,
                              const std::string &_user,
                              const std::string &_pass);

    /// \brief Notify the service with a http POST
    /// \param[in] _route on the web server
    /// \param[in] _json the data to send to the server
    public: void PostJsonData(const char* _route, const char *_json);

    /// \brief Returns the username
    /// \return The user name
    public: std::string GetUser() const;

    /// \brief a Request/Respone (can be used for GET and POST)
    /// \param[in] _requestUrl The request url.
    /// \param[in] _postStr The data to post
    /// \return The web server response
    private: std::string Request(const std::string &_requestUrl,
                                 const std::string &_postStr);

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
