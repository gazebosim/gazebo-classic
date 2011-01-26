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

/* Desc: Performance test for HTTP portal to libgazebo
 * Author: Brian Gerkey
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/time.h>

// These headers must be included prior to the libevent headers
#include <sys/types.h>
#include <sys/queue.h>

// libevent
#include <event.h>
#include <evhttp.h>

struct timeval g_t0, g_t1;
const char* g_uri;
size_t g_cnt;
double g_dt;
const size_t g_target_cnt = 1000;

// Callback that's invoked when the request is done and we have the
// response
void
cb(evhttp_request* req, void* arg)
{
  struct evhttp_connection* ec = (struct evhttp_connection*)arg;
  gettimeofday(&g_t1, NULL);
  if(req->response_code != 200)
  {
    printf("Got non-OK response code: %d\n", req->response_code);
    exit(1);
  }

  //printf("Response:\n%s", req->input_buffer->buffer);
  
  double dt = ((g_t1.tv_sec + g_t1.tv_usec/1e6) - 
               (g_t0.tv_sec + g_t0.tv_usec/1e6));
  g_dt += dt;
  //printf("Elapsed time: %.6f\n", dt);

  g_cnt++;
  if(g_cnt >= g_target_cnt)
  {
    printf("Total: %lu transactions / %.6fs = %.6f tran/s\n",
           g_cnt, g_dt, g_cnt / g_dt);
    exit(0);
  }

  gettimeofday(&g_t0, NULL);
  struct evhttp_request* er = evhttp_request_new(cb, (void*)ec);
  int ret = evhttp_make_request(ec, er, EVHTTP_REQ_GET, g_uri);
}

#define USAGE "USAGE: client <URI>"

int
main(int argc, char** argv)
{
  if(argc != 2)
  {
    puts(USAGE);
    exit(1);
  }

  g_uri = argv[1];

  struct evhttp_connection* ec;
  struct evhttp_request* er;

  event_init();

  ec = evhttp_connection_new("localhost", 8000);
  assert(ec);

  gettimeofday(&g_t0, NULL);
  er = evhttp_request_new(cb, (void*)ec);
  int ret = evhttp_make_request(ec, er, EVHTTP_REQ_GET, g_uri);
  event_dispatch();

  return 0;
}
