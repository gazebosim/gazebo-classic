/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#include "polhemus_driver/polhemus_driver.h"

#define MIN(a,b) ((a) > (b) ? (b) : (a))

struct polhemus_conn
{
  libusb_device_handle* usb_handle;
  int write_endpoint;
  int read_endpoint;
  int stop_requested;
  polhemus_pose_cb cb;
  pthread_t cb_thread;
};

polhemus_conn_t*
polhemus_connect_usb(int vendor_id, int product_id,
                     int write_endpoint, int read_endpoint)
{
  if(libusb_init(NULL) < 0)
  {
    fprintf(stderr, "libusb_init() failed\n");
    return NULL;
  }

  polhemus_conn_t* conn = (polhemus_conn_t*)malloc(sizeof(polhemus_conn_t));
  if(!conn)
  {
    fprintf(stderr, "malloc() failed\n");
    return NULL;
  }

  conn->usb_handle = libusb_open_device_with_vid_pid(NULL, vendor_id, product_id);
  if(!conn->usb_handle)
  {
    fprintf(stderr, "libusb_open_device_with_vid_pid() failed\n");
    free(conn);
    return NULL;
  }

  conn->write_endpoint = write_endpoint;
  conn->read_endpoint = read_endpoint;
  conn->cb = NULL;
  conn->stop_requested = 0;

  return conn;
}

void
polhemus_disconnect_usb(polhemus_conn_t* conn)
{
  libusb_close(conn->usb_handle);
  free(conn);
}

int
polhemus_write_usb(polhemus_conn_t* conn, unsigned char* data, int data_len)
{
  int ret;
  int sent;
  int timeout = 50;
  if((ret = libusb_bulk_transfer(conn->usb_handle, conn->write_endpoint,
                                 data, data_len, &sent, timeout)))
  {
    fprintf(stderr, "libusb_bulk_transfer() failed while writing: %d\n", ret);
    return ret;
  }
  return sent;
}

int
polhemus_read_usb(polhemus_conn_t* conn, unsigned char* data, int data_len)
{
  int ret;
  int received;
  int timeout = 50;
  if((ret = libusb_bulk_transfer(conn->usb_handle, conn->read_endpoint,
                                 data, data_len, &received, timeout)))
  {
    fprintf(stderr, "libusb_bulk_transfer() failed while reading: %d\n", ret);
    return ret;
  }
  return received;
}

int
polhemus_init_comm(polhemus_conn_t* conn, int max_retries)
{
  unsigned char buf[1024];
  useconds_t dt = 100000;
  int i;
  int len;
  // Start by trying to drain the bus, in case we're in continuous mode
  do
  {
    len = polhemus_read_usb(conn, buf, sizeof(buf));
  } while(len == LIBUSB_ERROR_OVERFLOW);
  for(i=0; i<max_retries; i++)
  {
    // Feed it P until it answers
    len = polhemus_write_usb(conn, (unsigned char*)"P", 1);
    if(len != 1)
      return -1;
    usleep(dt);
    len = polhemus_read_usb(conn, buf, sizeof(buf));
    if(len < 0)
      return -1;
    // If we got some data, any data, then we're happy
    if(len > 0)
    {
      // Send a 'U' to change units to cm (default is inches)
      len = polhemus_write_usb(conn, (unsigned char*)"U1\r", 3);
      if(len != 3)
        return -1;
      // No response expected
      return 0;
    }
  }
  // Timed out
  return -1;
}

void*
cb_func(void* arg)
{
  unsigned char buf[1024];
  polhemus_conn_t* conn = (polhemus_conn_t*)arg;
  for(;;)
  {
    if(conn->stop_requested)
      return NULL;
    int len = polhemus_read_usb(conn, buf, sizeof(buf));
    if(len > 0)
    {
      int id;
      double x, y, z, roll, pitch, yaw;
      if(sscanf((const char*)buf, "%d %lf %lf %lf %lf %lf %lf", &id, &x, &y, &z, &roll, &pitch, &yaw) != 7)
      {
        fprintf(stderr, "Failed to parse string:%s:\n", buf);
      }
      conn->cb(x, y, z, roll, pitch, yaw);
    }
    else
      usleep(1000);
  }
}

int
polhemus_start_continuous_mode(polhemus_conn_t* conn,
                               polhemus_pose_cb cb)
{
  if(polhemus_write_usb(conn, (unsigned char*)"C\r", 2) != 2)
  {
    fprintf(stderr, "Failed to put device in continuous mode.\n");
    return -1;
  }
  conn->cb = cb;
  if(pthread_create(&(conn->cb_thread), NULL, cb_func, (void*)conn))
  {
    fprintf(stderr, "Failed to create callback thread.\n");
    return -1;
  }
  return 0;
}

int
polhemus_stop_continuous_mode(polhemus_conn_t* conn)
{
  conn->stop_requested = 1;
  if(pthread_join(conn->cb_thread, NULL))
  {
    fprintf(stderr, "Failed to join callback thread\n");
    return -1;
  }
  // Stop continuous mode by getting one frame of data
  double x, y, z, roll, pitch, yaw;
  polhemus_get_pose(conn, &x, &y, &z, &roll, &pitch, &yaw, 10);
  return 0;
}

int
polhemus_get_pose(polhemus_conn_t* conn,
                  double* x, double* y, double* z,
                  double* roll, double* pitch, double* yaw,
                  int max_retries)
{
  unsigned char buf[1024];
  int i;
  // Send 'p' to get data back
  if(polhemus_write_usb(conn, (unsigned char*)"P", 1) != 1)
  {
    fprintf(stderr, "Failed to write data\n");
    return -1;
  }
  // Read until we get data
  int len;
  for(i=0; i<max_retries; i++)
  {
    len = polhemus_read_usb(conn, buf, sizeof(buf));
    if(len > 0)
      break;
    usleep(10000);
  }
  buf[len] = '\0';
  int id;
  if(sscanf((const char*)buf, "%d %lf %lf %lf %lf %lf %lf", &id, x, y, z, roll, pitch, yaw) != 7)
  {
    fprintf(stderr, "Failed to parse string:%s:\n", buf);
    return -1;
  }
  return 0;
}

int
polhemus_get_poses(polhemus_conn_t* conn,
                   polhemus_pose_t* poses,
                   int* num_poses, int max_retries)
{
  unsigned char buf[1024];
  int i;
  // Send 'p' to get data back
  if(polhemus_write_usb(conn, (unsigned char*)"P", 1) != 1)
  {
    fprintf(stderr, "Failed to write data\n");
    return -1;
  }
  // Read until we get data
  int len;
  for(i=0; i<max_retries; i++)
  {
    len = polhemus_read_usb(conn, buf, sizeof(buf));
    if(len > 0)
      break;
    if (i+1 < max_retries)
    {
      // sleep a bit
      usleep(10000);
    }
    else
    {
      // no more retries, failed
      return -1;
    }
  }
  buf[len] = '\0';
  // Read multiple poses, separated by 0x0a
  char* sep_idx = (char*)buf;
  int poses_read = 0;
  // fprintf(stderr, "0 sep_idx [%s] buf [%s] pr [%d] np [%d]\n",
  //         sep_idx, buf, poses_read, *num_poses);
  while((sep_idx != NULL) && 
        (sep_idx < (char*)buf+len-1) && 
        (poses_read < *num_poses))
  {
    int id;
    double x, y, z, roll, pitch, yaw;
    if(sscanf((const char*)sep_idx, "%d %lf %lf %lf %lf %lf %lf",
      &id, &x, &y, &z, &roll, &pitch, &yaw) != 7)
    {
      fprintf(stderr, "Failed to parse string:%s:\n", buf);
      return -1;
    }
    poses[poses_read].station_id = id;
    poses[poses_read].x = x;
    poses[poses_read].y = y;
    poses[poses_read].z = z;
    poses[poses_read].roll = roll;
    poses[poses_read].pitch = pitch;
    poses[poses_read].yaw = yaw;
    poses_read++;

    sep_idx = strchr((const char*)MIN(sep_idx+1, (char*)buf+len), 0x0a);
  }
  *num_poses = poses_read;
  return 0;
}

