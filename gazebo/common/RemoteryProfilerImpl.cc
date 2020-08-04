/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "RemoteryProfilerImpl.hh"
#include "gazebo/common/Console.hh"

using namespace gazebo;
using namespace common;

std::string rmtErrorToString(rmtError error) {
  switch (error) {
    case RMT_ERROR_NONE:
      return "none";
    case RMT_ERROR_RECURSIVE_SAMPLE:
      return "Not an error but an internal message to calling code";

    // System errors
    case RMT_ERROR_MALLOC_FAIL:
      return "Malloc call within remotery failed";
    case RMT_ERROR_TLS_ALLOC_FAIL:
      return "Attempt to allocate thread local storage failed";
    case RMT_ERROR_VIRTUAL_MEMORY_BUFFER_FAIL:
      return "Failed to create a virtual memory mirror buffer";
    case RMT_ERROR_CREATE_THREAD_FAIL:
      return "Failed to create a thread for the server";

    // Network TCP/IP socket errors
    case RMT_ERROR_SOCKET_INIT_NETWORK_FAIL:
      return "Network initialisation failure (e.g. on Win32, WSAStartup fails)"; //NOLINT
    case RMT_ERROR_SOCKET_CREATE_FAIL:
      return "Can't create a socket for connection to the remote viewer";
    case RMT_ERROR_SOCKET_BIND_FAIL:
      return "Can't bind a socket for the server";
    case RMT_ERROR_SOCKET_LISTEN_FAIL:
      return "Created server socket failed to enter a listen state";
    case RMT_ERROR_SOCKET_SET_NON_BLOCKING_FAIL:
      return "Created server socket failed to switch to a non-blocking state";
    case RMT_ERROR_SOCKET_INVALID_POLL:
      return "Poll attempt on an invalid socket";
    case RMT_ERROR_SOCKET_SELECT_FAIL:
      return "Server failed to call select on socket";
    case RMT_ERROR_SOCKET_POLL_ERRORS:
      return "Poll notified that the socket has errors";
    case RMT_ERROR_SOCKET_ACCEPT_FAIL:
      return "Server failed to accept connection from client";
    case RMT_ERROR_SOCKET_SEND_TIMEOUT:
      return "Timed out trying to send data";
    case RMT_ERROR_SOCKET_SEND_FAIL:
      return "Unrecoverable error occured while client/server tried to send data"; //NOLINT
    case RMT_ERROR_SOCKET_RECV_NO_DATA:
      return "No data available when attempting a receive";
    case RMT_ERROR_SOCKET_RECV_TIMEOUT:
      return "Timed out trying to receive data";
    case RMT_ERROR_SOCKET_RECV_FAILED:
      return "Unrecoverable error occured while client/server tried to receive data"; //NOLINT

    // WebSocket errors
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_NOT_GET:
      return "WebSocket server handshake failed, not HTTP GET";
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_NO_VERSION:
      return "WebSocket server handshake failed, can't locate WebSocket version"; //NOLINT
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_BAD_VERSION:
      return "WebSocket server handshake failed, unsupported WebSocket version";
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_NO_HOST:
      return "WebSocket server handshake failed, can't locate host";
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_BAD_HOST:
      return "WebSocket server handshake failed, host is not allowed to connect"; //NOLINT
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_NO_KEY:
      return "WebSocket server handshake failed, can't locate WebSocket key";
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_BAD_KEY:
      return "WebSocket server handshake failed, WebSocket key is ill-formed";
    case RMT_ERROR_WEBSOCKET_HANDSHAKE_STRING_FAIL:
      return "WebSocket server handshake failed, internal error, bad string code"; //NOLINT
    case RMT_ERROR_WEBSOCKET_DISCONNECTED:
      return "WebSocket server received a disconnect request and closed the socket"; //NOLINT
    case RMT_ERROR_WEBSOCKET_BAD_FRAME_HEADER:
      return "Couldn't parse WebSocket frame header";
    case RMT_ERROR_WEBSOCKET_BAD_FRAME_HEADER_SIZE:
      return "Partially received wide frame header size";
    case RMT_ERROR_WEBSOCKET_BAD_FRAME_HEADER_MASK:
      return "Partially received frame header data mask";
    case RMT_ERROR_WEBSOCKET_RECEIVE_TIMEOUT:
      return "Timeout receiving frame header";

    case RMT_ERROR_REMOTERY_NOT_CREATED:
      return "Remotery object has not been created";
    case RMT_ERROR_SEND_ON_INCOMPLETE_PROFILE:
      return "An attempt was made to send an incomplete profile tree to the client"; // NOLINT

    // CUDA error messages
    case RMT_ERROR_CUDA_DEINITIALIZED:
      return "This indicates that the CUDA driver is in the process of shutting down"; //NOLINT
    case RMT_ERROR_CUDA_NOT_INITIALIZED:
     return "This indicates that the CUDA driver has not been initialized with cuInit() or that initialization has failed"; //NOLINT
    case RMT_ERROR_CUDA_INVALID_CONTEXT:
      return "This most frequently indicates that there is no context bound to the current thread"; //NOLINT
    case RMT_ERROR_CUDA_INVALID_VALUE:
      return "This indicates that one or more of the parameters passed to the API call is not within an acceptable range of values"; //NOLINT
    case RMT_ERROR_CUDA_INVALID_HANDLE:
      return "This indicates that a resource handle passed to the API call was not valid"; //NOLINT
    case RMT_ERROR_CUDA_OUT_OF_MEMORY:
      return "The API call failed because it was unable to allocate enough memory to perform the requested operation"; //NOLINT
    case RMT_ERROR_ERROR_NOT_READY:
      return "This indicates that a resource handle passed to the API call was not valid"; //NOLINT

    // Direct3D 11 error messages
    case RMT_ERROR_D3D11_FAILED_TO_CREATE_QUERY:
      return "Failed to create query for sample";

    // OpenGL error messages
    case RMT_ERROR_OPENGL_ERROR:
      return "Generic OpenGL error, no need to expose detail since app will need an OpenGL error callback registered"; //NOLINT
    case RMT_ERROR_CUDA_UNKNOWN:
      return "Unknown CUDA error";
    default:
      return "Unknown remotery error";
  }
}

//////////////////////////////////////////////////
RemoteryProfilerImpl::RemoteryProfilerImpl()
{
  this->settings = rmt_Settings();

  // Always attempt to reuse the port
  this->settings->reuse_open_port = RMT_TRUE;

  auto port_c = std::getenv("RMT_PORT");
  if(port_c){
    std::string port = std::string(std::getenv("RMT_PORT"));
    if (!port.empty())
    {
      this->settings->port = std::stoul(port);
    }
  }

  auto queue_size_c = std::getenv("RMT_QUEUE_SIZE");
  if(queue_size_c){
    std::string queue_size = std::string(queue_size_c);
    if (!queue_size.empty())
    {
      this->settings->messageQueueSizeInBytes = std::stoul(queue_size);
    }
  } else {
    this->settings->messageQueueSizeInBytes = 2048 * 2048;
  }

  auto messages_per_update_c = std::getenv("RMT_MSGS_PER_UPDATE");
  if(messages_per_update_c){
    std::string messages_per_update = std::string(messages_per_update_c);
    if (!messages_per_update.empty())
    {
      this->settings->maxNbMessagesPerUpdate = std::stoul(messages_per_update);
    }
  } else {
    this->settings->maxNbMessagesPerUpdate = 10;
  }

  auto sleep_between_updates_c = std::getenv("RMT_SLEEP_BETWEEN_UPDATES");
  if (sleep_between_updates_c) {
    std::string sleep_between_updates = std::string(sleep_between_updates_c);
    if (sleep_between_updates.empty())
    {
      this->settings->msSleepBetweenServerUpdates =
          std::stoul(sleep_between_updates);
    }
  } else {
    this->settings->msSleepBetweenServerUpdates = 10;
  }

  this->settings->input_handler_context = this;
  this->settings->input_handler = [](const char *_text, void *_context) {
    static_cast<RemoteryProfilerImpl *>(_context)->HandleInput(_text);
  };

  gzmsg << "Staring profiler impl: Remotery" <<
     " (port: " << this->settings->port << ")" << std::endl;
  rmtError error;
  error = rmt_CreateGlobalInstance(&this->rmt);

  if (RMT_ERROR_NONE != error)
  {
    // gzerr << "Error launching Remotery: " <<
    //     rmtErrorToString(error) << std::endl;
    this->rmt = nullptr;
  }
}

//////////////////////////////////////////////////
RemoteryProfilerImpl::~RemoteryProfilerImpl()
{
  if (this->rmt)
    rmt_DestroyGlobalInstance(this->rmt);
}

//////////////////////////////////////////////////
std::string RemoteryProfilerImpl::Name() const
{
  return "ign_profiler_remotery";
}

//////////////////////////////////////////////////
void RemoteryProfilerImpl::SetThreadName(const char *_name)
{
  rmt_SetCurrentThreadName(_name);
}

//////////////////////////////////////////////////
void RemoteryProfilerImpl::LogText(const char *_text)
{
  rmt_LogText(_text);
}

//////////////////////////////////////////////////
void RemoteryProfilerImpl::BeginSample(const char *_name, uint32_t *_hash)
{
  _rmt_BeginCPUSample(_name, RMTSF_Aggregate, _hash);
}

//////////////////////////////////////////////////
void RemoteryProfilerImpl::EndSample()
{
  rmt_EndCPUSample();
}

//////////////////////////////////////////////////
void RemoteryProfilerImpl::HandleInput(const char *_text)
{
  (void) _text;
}
