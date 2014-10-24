
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


#ifndef _MOOC_REST_API_HH_
#define _MOOC_REST_API_HH_

#include <list>
#include "MOOCException.hh"

namespace gazebo
{

  class MOOCRestApi
  {
    /// \brief ctor
    public: MOOCRestApi();

    /// \brief dtor
    public: virtual ~MOOCRestApi();

    /// \brief Connects to the MOOC. 
    /// \returns MOOC version information
    public: std::string Login(const char* url, const char* route, const char* user, const char* pass);

    /// \brief Notify the MOOC of a learning event
    public: void PostLearningEvent(const char* route, const char *json);
   
    /// \brief a Request/Response
    /// \return the response
    private: std::string Request(const char* request, const char *postStr = NULL);
    
    /// \brief Login information: MOOC host url
    private: std::string url;

    /// \brief Login information: MOOC username 
    private: std::string user;

    /// \brief Login information: MOOC password
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
