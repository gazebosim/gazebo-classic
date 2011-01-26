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
// Compile with:
//   g++ -o http http.cc -levent

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

// These headers must be included prior to the libevent headers
#include <sys/types.h>
#include <sys/queue.h>

// libevent
#include <event.h>
#include <evhttp.h>

// Callback that's invoked when the request is done and we have the
// response
void
cb(evhttp_request* req, void* arg)
{
  printf("in client cb, got code:%d\n", req->response_code);
  printf("response len: %d %d %d\n", 
         req->input_buffer->misalign,
         req->input_buffer->totallen,
         req->input_buffer->off);
  printf(":%s:%s:\n",
         req->input_buffer->buffer,
         req->input_buffer->orig_buffer);
  exit(0);
}

int
main( int argc, char* argv[] )
{
  // initialize libevent
  event_init();
  
  struct evhttp_connection* ec( evhttp_connection_new("localhost", 8000) );
  assert(ec);
  
  struct evhttp_request* er(evhttp_request_new(cb, NULL));
  assert( er );
 
  char* request = (char*)"";

  if( argc == 1 && argv[1] )
    request = argv[1];

  int ret = evhttp_make_request(ec, er, EVHTTP_REQ_GET, request );
  printf("ret: %d\n", ret);

  event_dispatch();

  return 0;
}
