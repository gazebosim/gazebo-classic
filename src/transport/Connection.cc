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
#include "transport/IOManager.hh"
#include "transport/Connection.hh"

using namespace gazebo;
using namespace transport;

Connection::Connection(boost::asio::io_service &io_service)
  : socket(io_service)
{
}

Connection::~Connection()
{
}

boost::asio::ip::tcp::socket &Connection::GetSocket()
{
  return this->socket;
}

void Connection::Connect(const std::string &host, const std::string &service)
{
  // Resolve the host name into an IP address
  boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::query query(host, service);
  boost::asio::ip::tcp::resolver::iterator endpoint_iter = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end;
  //boost::asio::ip::tcp::endpoint endpoint = *endpoint_iter;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iter != end)
  {
    socket.close();
    socket.connect(*endpoint_iter++, error);
  }
  if (error)
    throw boost::system::system_error(error);
}
