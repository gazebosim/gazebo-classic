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
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <curl/curl.h>
#include <inttypes.h>

#include "RestApi.hh"

using namespace gazebo;

// This code is adapted from the curl C examples
// (http://curl.haxx.se/libcurl/c/)
// mostly modified to adhere to the Gazebo code check tool.
// It implements the following features:
//  - HTML POST, sending and receiving data
//  - Authentication (Basic Auth)
//  - https (SSL, but accepting non signed certificates)

// You can enable this flag to get more curl details
// about each request (will provide  SSL negociation and TCP dumps of the
// requests and responses)
bool trace_requests = false;

struct data
{
  // 1 or 0
  char trace_ascii;
};

/////////////////////////////////////////////////
static void DumpRequest(const char *_text,
          FILE *_stream,
          unsigned char *_ptr,
          size_t _size,
          char _nohex)
{
  size_t i;
  size_t c;
  unsigned int width = 0x10;

  if (_nohex)
  {
    // without the hex output, we can fit more on screen
    width = 0x40;
  }
  int64_t s = _size;
  fprintf(_stream,
    "%s, %10.10" PRId64 " bytes (0x%8.8" PRIx64 ")\n", _text, s, s);

  for (i = 0; i < _size; i += width)
  {
    fprintf(_stream, "%4.4lx: ", i);
    if (!_nohex)
    {
      // hex not disabled, show it
      for (c = 0; c < width; ++c)
      {
        if (i+c < _size)
          fprintf(_stream, "%02x ", _ptr[i+c]);
        else
          fputs("   ", _stream);
      }
    }

    for (c = 0; (c < width) && (i+c < _size); ++c)
    {
      // check for 0D0A; if found, skip past and start a new line of output
      if (_nohex && (i+c+1 < _size) && _ptr[i+c] == 0x0D && _ptr[i+c+1] == 0x0A)
      {
        i+=(c+2-width);
        break;
      }
      fprintf(_stream, "%c",
              (_ptr[i+c] >= 0x20) && (_ptr[i+c] < 0x80)?_ptr[i+c]:'.');
      // check again for 0D0A, to avoid an extra \n if it's at width
      if (_nohex && (i+c+2 < _size) && _ptr[i+c+1] == 0x0D &&
          _ptr[i+c+2] == 0x0A)
      {
        i+=(c+3-width);
        break;
      }
    }
    fputc('\n', _stream);
  }
  fflush(_stream);
}

/////////////////////////////////////////////////
// Callback given to curl that outputs data about the request
static int TraceRequest(CURL *_handle,
                        curl_infotype _type,
                        char *_data,
                        size_t _size,
                        void *_userp)
{
  struct data *config = (struct data *)_userp;
  const char *text;

  // prevent compiler warning
  (void)_handle;

  switch (_type)
  {
    case CURLINFO_TEXT:
      if (trace_requests)
        fprintf(stderr, "== Info: %s", _data);
    // in case a new one is introduced to shock us
    default:
      return 0;
    case CURLINFO_HEADER_OUT:
      text = "=> Send header";
      break;
    case CURLINFO_DATA_OUT:
      text = "=> Send data";
      break;
    case CURLINFO_SSL_DATA_OUT:
      text = "=> Send SSL data";
      break;
    case CURLINFO_HEADER_IN:
      text = "<= Recv header";
      break;
    case CURLINFO_DATA_IN:
      text = "<= Recv data";
      break;
    case CURLINFO_SSL_DATA_IN:
      text = "<= Recv SSL data";
      break;
  }

  if (trace_requests)
  {
    DumpRequest(text, stderr,
        (unsigned char *)(_data), _size, config->trace_ascii);
  }
  return 0;
}

// private data structure used to
// read libcurl response
struct MemoryStruct {
  char *memory;
  size_t size;
};

/////////////////////////////////////////////////
// callback for libcurl when data is read from http response
static size_t WriteMemoryCallback(void *_contents,
                                  size_t _size,
                                  size_t _nmemb,
                                  void *_userp)
{
  size_t realsize = _size * _nmemb;
  struct MemoryStruct *mem = (struct MemoryStruct *)_userp;
  size_t newsize = mem->size + realsize + 1;
  mem->memory = static_cast<char *> (realloc(mem->memory, newsize));
  if (mem->memory == NULL)
  {
    // out of memory!
    gzerr << "not enough memory (realloc returned NULL)" << std::endl;
    return 0;
  }
  memcpy(&(mem->memory[mem->size]), _contents, realsize);
  mem->size += realsize;
  mem->memory[mem->size] = 0;
  return realsize;
}

/////////////////////////////////////////////////
RestApi::RestApi()
  :isLoggedIn(false)
{
}

/////////////////////////////////////////////////
RestApi::~RestApi()
{
  curl_global_cleanup();
}

/////////////////////////////////////////////////
void RestApi::PostJsonData(const char *_route, const char *_json)
{
  Post post;
  post.route = _route;
  post.json = _json;
  {
    boost::mutex::scoped_lock lock(this->postsMutex);
    this->posts.push_back(post);
  }
  this->SendUnpostedPosts();
}

/////////////////////////////////////////////////
std::string RestApi::Login(const std::string &_urlStr,
                           const std::string &_route,
                           const std::string &_userStr,
                           const std::string &_passStr)
{
  this->isLoggedIn = false;
  this->url = _urlStr;
  this->user = _userStr;
  this->pass = _passStr;

  // at this point we want to test the (user supplied) login data
  // so we're hitting the server on the login route ('/login')
  this->loginRoute = _route;
  std::string resp;

  gzmsg << "login route: " << this->loginRoute << std::endl;
  resp = this->Request(loginRoute, "");
  gzmsg << "login response: " << resp << std::endl;

  this->isLoggedIn = true;
  this->SendUnpostedPosts();
  return resp;
}

/////////////////////////////////////////////////
void RestApi::Logout()
{
  this->isLoggedIn = false;
  gzmsg << "Logout" << std::endl;
}

/////////////////////////////////////////////////
void RestApi::SendUnpostedPosts()
{
  if (this->isLoggedIn)
  {
    while (!this->posts.empty())
    {
      Post post;
      {
        boost::mutex::scoped_lock lock(this->postsMutex);
        post = this->posts.front();

        //  You can generate a similar request on the cmd line like so:
        //  curl --verbose --connect-timeout 5 -X POST
        //    -H \"Content-Type: application/json \" -k --user"
        this->Request(post.route, post.json);
        this->posts.pop_front();
      }
    }
  }
  else
  {
    gzmsg << posts.size() << " post(s) queued to be sent" << std::endl;
  }
}

/////////////////////////////////////////////////
std::string RestApi::GetUser() const
{
  return this->user;
}

/////////////////////////////////////////////////
std::string RestApi::Request(const std::string &_reqUrl,
                             const std::string &_postJsonStr)
{
  if (this->url.empty())
    throw RestException("A URL must be specified for web service");

  if (this->user.empty())
  {
    std::string e = "No user specified for the web service. Please login.";
    throw RestException(e.c_str());
  }
  // build full url (with server)
  std::string path = url + _reqUrl;
  CURL *curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_URL, path.c_str() );

  // in case things go wrong
  if (trace_requests)
  {
    gzmsg << "RestApi::Request" << std::endl;
    gzmsg << "  path: " << path << std::endl;
    gzmsg << "  data: " << _postJsonStr << std::endl;
    gzmsg << std::endl;

    struct data config;
    config.trace_ascii = 1;  //  enable ascii tracing
    curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, TraceRequest);
    curl_easy_setopt(curl, CURLOPT_DEBUGDATA, &config);

    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curl, CURLOPT_HEADER, 1L);
  }

  struct MemoryStruct chunk;
  // will be grown as needed by the realloc above
  chunk.memory = static_cast<char*>(malloc(1));
  chunk.size = 0;            // no data at this point
  bool secure = false;
  if (!secure)
  {
    // skip peer verification
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    // skip host verification
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
  }

  // send all data to this function
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
  //  we pass our 'chunk' struct to the callback function
  curl_easy_setopt(curl,
                   CURLOPT_WRITEDATA,
                   static_cast<void *>(&chunk));

  // some servers don't like requests that are made without a user-agent
  // field, so we provide one
  curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

  // set user name and password for the authentication
  curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
  std::string userpass = this->user + ":" + this->pass;
  curl_easy_setopt(curl, CURLOPT_USERPWD, userpass.c_str());

  // connection timeout 10 sec
  curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10);

  // is this a POST?
  struct curl_slist *slist = NULL;
  if (!_postJsonStr.empty())
  {
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 0L);  // disable PUT
    curl_easy_setopt(curl, CURLOPT_POST, 1);  // enable POST
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, _postJsonStr.c_str());

    slist = curl_slist_append(slist, "Content-Type: application/json");
    slist = curl_slist_append(slist, "charsets: utf-8");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, slist);
  }

  CURLcode res;
  res = curl_easy_perform(curl);

  // get HTTP response code
  int64_t http_code = 0;

  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

  curl_easy_cleanup(curl);
  if (res != CURLE_OK)
  {
    gzerr << "Request to " << url << " failed: "
          << curl_easy_strerror(res) << std::endl;
    throw RestException(curl_easy_strerror(res));
  }
  // copy the data into a string
  std::string response(chunk.memory, chunk.size);

  if (http_code != 200)
  {
    gzerr << "Request to " << url << " error: " << response << std::endl;
    throw RestException(response.c_str());
  }
  // clean up
  curl_slist_free_all(slist);
  if (chunk.memory)
    free(chunk.memory);
  return response;
}
