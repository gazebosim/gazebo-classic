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
//   g++ -o httpd httpd.cc -levent

#include <stdio.h>
#include <assert.h>

// These headers must be included prior to the libevent headers
#include <sys/types.h>
#include <sys/queue.h>

// libevent
#include <event.h>
#include <evhttp.h>

void
cb(evhttp_request* req, void* arg)
{
  printf("in server cb for uri:%s:\n", req->uri);
  struct evbuffer* eb = evbuffer_new();
  assert(eb);
  evbuffer_add_printf(eb, "response for:%s:\n", req->uri);
  evhttp_send_reply(req, 200, "foo", eb);
  evbuffer_free(eb);
}

int
main(void)
{
  struct evhttp* eh;

  event_init();

  eh = evhttp_start("localhost", 7000);
  assert(eh);

  evhttp_set_gencb(eh, cb, NULL);
  //evhttp_set_cb(eh, "/foo", cb, NULL);

  event_dispatch();

  return 0;
}
