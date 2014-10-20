#include <stdio.h>
#include <unistd.h>

#include "polhemus_driver/polhemus_driver.h"

int
main(void)
{
  polhemus_conn_t* conn;

  if(!(conn = polhemus_connect_usb(LIBERTY_HS_VENDOR_ID,
                                   LIBERTY_HS_PRODUCT_ID,
                                   LIBERTY_HS_WRITE_ENDPOINT,
                                   LIBERTY_HS_READ_ENDPOINT)))
  {
    fprintf(stderr, "Failed to connect\n");
    return -1;
  }

  if(polhemus_init_comm(conn, 10))
  {
    fprintf(stderr, "Failed to initialize comms\n");
    return -1;
  }

  int num_poses = 8;
  polhemus_pose_t poses[8];
  for(;;)
  {
    polhemus_get_poses(conn, poses, &num_poses, 10);
    //printf("Received %d poses\n", num_poses);
    int i;
    for(i=0; i<num_poses; i++)
      printf("%d %lf %lf %lf %lf %lf %lf\n", poses[i].station_id, 
             poses[i].x, poses[i].y, poses[i].z,
             poses[i].roll, poses[i].pitch, poses[i].yaw);
  }

  polhemus_disconnect_usb(conn);
  return 0;
}
