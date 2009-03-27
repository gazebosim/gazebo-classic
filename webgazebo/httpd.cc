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
