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
main(void)
{
  struct evhttp_connection* ec;
  struct evhttp_request* er;

  event_init();

  ec = evhttp_connection_new("localhost", 8000);
  assert(ec);

  er = evhttp_request_new(cb, NULL);

  int ret = evhttp_make_request(ec, er, EVHTTP_REQ_GET, "/foo");
  printf("ret: %d\n", ret);

  event_dispatch();

  return 0;
}
