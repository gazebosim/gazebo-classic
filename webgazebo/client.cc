/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
    printf("Total: %d transactions / %.6fs = %.6f tran/s\n",
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
