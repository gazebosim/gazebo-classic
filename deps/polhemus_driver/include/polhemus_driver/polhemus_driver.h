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
#ifndef _POLHEMUS_DRIVER_H_
#define _POLHEMUS_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

// Magic USB numbers that are model-specific
#define LIBERTY_HS_VENDOR_ID 0x0f44
#define LIBERTY_HS_PRODUCT_ID 0xff20
#define LIBERTY_HS_WRITE_ENDPOINT 0x04
#define LIBERTY_HS_READ_ENDPOINT 0x88

struct polhemus_conn;
typedef struct polhemus_conn polhemus_conn_t;

typedef struct
{
  int station_id;
  double x, y, z, roll, pitch, yaw;
} polhemus_pose_t;

typedef void(*polhemus_pose_cb)(double x, double y, double z,
                                double roll, double pitch, double yaw);


polhemus_conn_t* polhemus_connect_usb(int vendor_id, int product_id,
                                      int write_endpoint, int read_endpoint);
void polhemus_disconnect_usb(polhemus_conn_t* conn);
int polhemus_write_usb(polhemus_conn_t* conn, unsigned char* data, int data_len);
int polhemus_read_usb(polhemus_conn_t* conn, unsigned char* data, int data_len);
int polhemus_init_comm(polhemus_conn_t* conn, int max_retries);
int polhemus_get_pose(polhemus_conn_t* conn,
                      double* x, double* y, double* z,
                      double* roll, double* pitch, double* yaw,
                      int max_retries);
/*
 * polhemus_get_poses
 * num_poses[in] max poses to read
 * num_poses[out] actual number of poses read from sensor
*/
int polhemus_get_poses(polhemus_conn_t* conn,
                       polhemus_pose_t* poses,
                       int* num_poses, int max_retries);
int polhemus_start_continuous_mode(polhemus_conn_t* conn,
                                   polhemus_pose_cb cb);
int polhemus_stop_continuous_mode(polhemus_conn_t* conn);

#ifdef __cplusplus
}
#endif

#endif
